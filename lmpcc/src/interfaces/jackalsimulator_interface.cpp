#include "interfaces/jackalsimulator_interface.h"

#include <lmpcc/lmpcc_controller.h>
#include <ros_tools/helpers.h>

JackalSimulatorInterface::JackalSimulatorInterface(ros::NodeHandle &nh, MPCC *controller, predictive_configuration *config, SolverInterface *solver_interface_ptr)
    : Interface(nh, controller, config, solver_interface_ptr)
{
  ROS_WARN("Initializing Jackal Interface...");
  obstacle_markers_.reset(new RosTools::ROSMarkerPublisher(nh, "lmpcc/received_obstacles", config_->target_frame_, 20)); // 3500)); // was 1800
  road_markers_.reset(new RosTools::ROSMarkerPublisher(nh, "lmpcc/simulation/road", config_->target_frame_, 20));        // 3500)); //
                                                                                                                         // was 1800

  monitor_.reset(new Monitor(nh)); // Monitor signals

  // Subscribers for sensor data
  state_sub_ = nh.subscribe(config_->robot_state_topic_, 1, &JackalSimulatorInterface::StateCallBack, this);
  velocity_sub_ = nh.subscribe(config_->robot_velocity_topic_, 1, &JackalSimulatorInterface::VelocityCallback, this);

  // Publisher for vehicle command
  command_pub_ = nh.advertise<geometry_msgs::Twist>(config_->cmd_, 1);

  reset_sub_ = nh.subscribe(config_->reset_topic_, 1, &JackalSimulatorInterface::ResetCallback, this);

  // Subscribe to waypoints
  waypoints_sub_ = nh.subscribe(config_->waypoint_topic_, 1, &Interface::RoadmapWaypointsCallback, (Interface *)this);
  goal_sub_ = nh.subscribe("/nn_jackal/goal", 1, &Interface::GoalCallback, (Interface *)this);

  // Service clients for resetting
  reset_simulation_pub_ = nh.advertise<std_msgs::Empty>(config_->reset_environment_topic_.c_str(), 1);

  reset_simulation_client_ = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
  reset_ekf_client_ = nh.serviceClient<robot_localization::SetPose>("/set_pose");
  reset_pose_msg_.request.pose.pose.pose.orientation.x = 0;
  reset_pose_msg_.request.pose.pose.pose.orientation.y = 0;
  reset_pose_msg_.request.pose.pose.pose.orientation.z = 0;
  reset_pose_msg_.request.pose.pose.pose.orientation.w = 1;
  reset_pose_msg_.request.pose.pose.pose.position.x = 0.;
  reset_pose_msg_.request.pose.pose.pose.position.y = 0.;

  // Regular obstacle subscribers
  obstacle_sub_ = nh.subscribe(config_->obs_state_topic_, 1, &JackalSimulatorInterface::ObstacleCallBack, this);
  trajectory_prediction_sub_ = nh.subscribe("/pedestrian_simulator/trajectory_predictions", 1, &JackalSimulatorInterface::ObstaclePredictionsCallback, this);

  // Subscribers for high level data
  // pedsim_sub_ = nh.subscribe("/pedsim_simulator/simulated_agents", 2, &Interface::PedsimObstacleCallBack, (Interface *)this);

  collision_sub_ = nh.subscribe("/lmpcc/collision_detected", 2, &JackalSimulatorInterface::CollisionCallback, this);

  simulation_tool_.reset(new RosTools::SimulationTool(config_->reset_environment_topic_, 3.0, config_->number_of_experiments_));

  N_pub_ = nh.advertise<std_msgs::Float64>("pedestrian_simulator/N", 1);
  dt_pub_ = nh.advertise<std_msgs::Float64>("pedestrian_simulator/dt", 1);
  hz_pub_ = nh.advertise<std_msgs::Float64>("pedestrian_simulator/Hz", 1);

  /* For JSK visuals */
  signal_publishers_.emplace_back(nh, "acceleration");
  signal_publishers_.emplace_back(nh, "velocity");
  signal_publishers_.emplace_back(nh, "ang_velocity");

  monitor_->MarkExpected("State");
  monitor_->MarkExpected("Predictions");
  monitor_->MarkExpected("Partitions");

  t_last_reset_ = ros::Time::now();

  LMPCC_WARN("Jackal Interface Initialized");
}

void JackalSimulatorInterface::ActuateNow()
{
  // Publish the command
  command_pub_.publish(control_msg_);

  // Publish signals for visuals
  signal_publishers_[0].Publish(solver_interface_ptr_->control_inputs_.acceleration);
  signal_publishers_[1].Publish(solver_interface_ptr_->control_inputs_.velocity);
  signal_publishers_[2].Publish(solver_interface_ptr_->control_inputs_.rot_velocity);
}

void JackalSimulatorInterface::Actuate()
{
  monitor_->PrintStatus();
  control_msg_ = geometry_msgs::Twist();

  control_msg_.linear.x = solver_interface_ptr_->control_inputs_.velocity;
  control_msg_.angular.z = solver_interface_ptr_->control_inputs_.rot_velocity;

  // Publish signals for visuals
  // signal_publishers_[0].Publish(solver_interface_ptr_->control_inputs_.acceleration);
  // signal_publishers_[1].Publish(solver_interface_ptr_->control_inputs_.velocity);
  // signal_publishers_[2].Publish(solver_interface_ptr_->control_inputs_.rot_velocity);
}

void JackalSimulatorInterface::ActuateBrake(double deceleration)
{
  monitor_->PrintStatus();

  // deceleration = std::abs(deceleration);
  control_msg_ = geometry_msgs::Twist();

  double velocity_after_braking;
  // if (controller_->velocity_ >= 0.)
  // {
  velocity_after_braking = controller_->velocity_ - deceleration * (1.0 / ((double)config_->clock_frequency_)); // Brake with the given deceleration
  control_msg_.linear.x = std::max(velocity_after_braking, 0.);                                                 // Don't drive backwards when braking
  // }
  // else
  // {
  //     velocity_after_braking = controller_->velocity_ + deceleration * (1.0 / ((double)config_->clock_frequency_));
  //     // Brake with the given deceleration control_msg_.linear.x = std::min(velocity_after_braking, 0.); // Don't
  //     drive backwards when braking
  // }
  // control_msg_.linear.x = 0.;
  control_msg_.angular.z = 0.0;

  // Publish signals for visuals
  // signal_publishers_[0].Publish(deceleration);
  // signal_publishers_[1].Publish(control_msg_.linear.x);
  // signal_publishers_[2].Publish(control_msg_.angular.z);

  // Publish the command
  // command_pub_.publish(control_msg_);
}

void JackalSimulatorInterface::Reset()
{
  t_last_reset_ = ros::Time::now(); // Prevent time-outs

  for (int j = 0; j < 5; j++)
  {
    ActuateBrake(3.0);
    ros::Duration(1.0 / config_->clock_frequency_).sleep();
  }

  // reset_jackal_client_.call(link_state_msg);

  std_msgs::Float64 N_msg, dt_msg, hz_msg;

  // HOMOTOPY (2x the horizon)
  N_msg.data = config_->N_pedestrians_;
  N_pub_.publish(N_msg);

  dt_msg.data = solver_interface_ptr_->DT;
  dt_pub_.publish(dt_msg);

  hz_msg.data = config_->clock_frequency_;
  hz_pub_.publish(hz_msg);

  reset_simulation_client_.call(reset_msg_);
  reset_ekf_client_.call(reset_pose_msg_);
  reset_simulation_pub_.publish(std_msgs::Empty());

  ros::Duration(1.0 / config_->clock_frequency_).sleep();
  t_last_reset_ = ros::Time::now(); // Set the last reset time
}

/* CALLBACKS */
void JackalSimulatorInterface::ResetCallback(const std_msgs::Empty &msg) { controller_->OnReset(); }

void JackalSimulatorInterface::VelocityCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  // LMPCC_WARN("Velocity Callback");
  // solver_interface_ptr_->State().set_v(std::sqrt(std::pow(msg->twist.twist.linear.x, 2) +
  // std::pow(msg->twist.twist.linear.y, 2))); // for shifting the current coordinates to the center of mass
  // solver_interface_ptr_->State().set_w(msg->twist.twist.angular.z); // for shifting the current coordinates to the
  // center of mass
}

void JackalSimulatorInterface::StateCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  PROFILE_FUNCTION(); // To track when the state is received
  LMPCC_INFO("State Callback");
  monitor_->MarkReceived("State");

  // Check if this state was received during a reset, if so ignore it and wait for a correct value!
  if (t_last_reset_ + ros::Duration(0.005) > msg->header.stamp)
    return;

  // Update the frame
  config_->target_frame_ = msg->header.frame_id;
  t_state_received_ = msg->header.stamp;

  // Check if the Jackal flipped over (it happens)
  // std::cout << msg->pose.orientation.x << ", " << msg->pose.orientation.y << std::endl;
  if (std::abs(msg->pose.orientation.x) > (M_PI / 8.) || std::abs(msg->pose.orientation.y) > (M_PI / 8.))
  {
    LMPCC_SUCCESS_ALWAYS("Jackal Simulation Interface: Detected Flipped Jackal. Resetting...");
    std_msgs::Empty empty_msg;
    ResetCallback(empty_msg);
    t_last_reset_ = ros::Time::now();
    return;
  }

  // Update the states
  solver_interface_ptr_->State().set_x(msg->pose.position.x); // for shifting the current coordinates to the center of
                                                              // mass
  solver_interface_ptr_->State().set_y(msg->pose.position.y);
  solver_interface_ptr_->State().set_psi(msg->pose.orientation.z);
  controller_->velocity_ = msg->pose.position.z;
  data_.velocity_ = msg->pose.position.z;
  solver_interface_ptr_->State().set_v(msg->pose.position.z); // When v is a state
  data_.state_received_time_ = msg->header.stamp;

  // Plots a Axis to display the path variable location

  if (t_last_tf_ + ros::Duration(0.01) < ros::Time::now())
  {
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    t_last_tf_ = transformStamped.header.stamp;
    transformStamped.header.frame_id = config_->target_frame_;
    transformStamped.child_frame_id = "robot_state";

    transformStamped.transform.translation.x = solver_interface_ptr_->State().x();
    transformStamped.transform.translation.y = 0.; // solver_interface_ptr_->State().y();
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;

    path_pose_pub_.sendTransform(transformStamped);
  }

  if (config_->debug_output_)
    solver_interface_ptr_->State().print();

  PlotRoad();

  controller_->OnStateReceived();

  // Override the objective reached for experiments
  if (msg->pose.position.x > 25)
    controller_->SetExternalObjectiveReached();

  // Reset if some time has passed after the last reset and we reached our goal OR if the robot got stuck
  if (((ros::Time::now() - t_last_reset_).sec > 1.0 && controller_->objective_reached_) || (ros::Time::now() - t_last_reset_).sec > config_->time_out_)
  {
    LMPCC_SUCCESS_ALWAYS("Jackal Simulation Interface: Jackal reached the end of the path. Resetting...");
    std_msgs::Empty empty_msg;
    ResetCallback(empty_msg);
    t_last_reset_ = ros::Time::now();
  }
}

// Callback for Gaussian obstacle information (i.e., mean and covariance for all k)
// We need to associate these predictions to the obstacles here...
void JackalSimulatorInterface::ObstaclePredictionsCallback(const lmpcc_msgs::obstacle_array &msg)
{
  PROFILE_FUNCTION(); // So that we can see when it arrives

  LMPCC_INFO("JackalSimulatorInterface::ObstaclePredictionsCallback received " << msg.obstacles.size() << " obstacle predictions");
  monitor_->MarkReceived("Predictions");

  prediction_msg_ = msg; // Save the data

  // Load all obstacles into our format
  data_.dynamic_obstacles_.clear();

  int disc_counter = 0; /** Track the number of obstacle discs */

  // Note: We loop until we have all the DISCS that we can handle
  for (auto &obstacle : msg.obstacles)
  {
    // Pedestrians as a single disc
    data_.dynamic_obstacles_.emplace_back(obstacle.id, disc_counter, Eigen::Vector2d(obstacle.pose.position.x, obstacle.pose.position.y), config_->r_VRU_);
    // std::sqrt(//std::pow(object.shape.dimensions[0] / 2., 2.) + std::pow(object.shape.dimensions[1] / 2., 2.)));

    data_.dynamic_obstacles_.back().pose_ = obstacle.pose;

    disc_counter++;
  }

  LMPCC_INFO_STREAM("Carla Interface: Converted " << data_.dynamic_obstacles_.size() << " obstacles to LMPCC format");

  PlotAllObstacles();

  PostProcessObstacles();

  // Load predictions into the obstacles
  for (auto &obstacle_prediction : msg.obstacles)
  {
    for (auto &obstacle : data_.dynamic_obstacles_)
    {
      // Association based on id
      if (obstacle_prediction.id == obstacle.id_)
      {
        obstacle.LoadPredictions(obstacle_prediction);
        break;
      }
    }
  }

  // Validate the predictions
  if (config_->debug_output_)
  {
    for (auto &obstacle : data_.dynamic_obstacles_)
    {
      if (obstacle.prediction_.gaussians[0].mean.poses.size() == 0)
      {
        LMPCC_WARN("Predictions of Obstacle " << obstacle.id_ << " are empty!");
      }
    }
  }

  controller_->OnObstaclesReceived(); // Process in the modules if necessary
}

// Create a list of dynamic obstacles
void JackalSimulatorInterface::CreateObstacleList()
{
  data_.dynamic_obstacles_.clear();

  int disc_id = 0;
  for (auto &object : obstacle_msg_.objects)
  {
    // Pedestrians as a single disc
    data_.dynamic_obstacles_.emplace_back(object.id, disc_id, Eigen::Vector2d(object.pose.position.x, object.pose.position.y), config_->r_VRU_);

    data_.dynamic_obstacles_.back().pose_ = object.pose;

    disc_id++;
  }

  PlotAllObstacles();
}

void JackalSimulatorInterface::CollisionCallback(const std_msgs::Float64 &msg)
{
  data_.intrusion_ = msg.data;
}

void JackalSimulatorInterface::PlotRoad()
{
  RosTools::ROSLine &line = road_markers_->getNewLine();
  line.setScale(0.05, 0.05);
  line.setColor(0., 0., 0.);

  geometry_msgs::Point p1, p2;
  p1.x = 0.;
  p1.y = config_->road_width_left_;
  p2.x = config_->ref_x_.back();
  p2.y = config_->road_width_left_;
  line.addLine(p1, p2);

  p1.x = 0.;
  p1.y = -config_->road_width_right_;
  p2.x = config_->ref_x_.back();
  p2.y = -config_->road_width_right_;
  line.addLine(p1, p2);

  road_markers_->publish();
}

void JackalSimulatorInterface::ObstacleCallBack(const derived_object_msgs::ObjectArray &msg)
{
  LMPCC_INFO("Obstacle Callback with " << msg.objects.size() << " objects");
  obstacle_msg_ = msg;

  // controller_->OnObstaclesReceived(); Update this when predictions are received
}

void JackalSimulatorInterface::WaypointsCallback(const nav_msgs::Path &msg)
{
  // NOT USED
  // if (config_->debug_output_)
  //     ROS_WARN_STREAM("Waypoint callback with " << msg.poses.size() << " waypoints");

  // controller_->reference_path_->waypoints_size_ = msg.poses.size();

  // controller_->reference_path_->x_.resize(controller_->reference_path_->waypoints_size_);
  // controller_->reference_path_->y_.resize(controller_->reference_path_->waypoints_size_);
  // controller_->reference_path_->psi_.resize(controller_->reference_path_->waypoints_size_);

  // for (size_t i = 0; i < controller_->reference_path_->waypoints_size_; i++)
  // {
  //     controller_->reference_path_->psi_.push_back(RosTools::quaternionToAngle(msg.poses[i].pose));
  //     controller_->reference_path_->x_.push_back(msg.poses[i].pose.position.x);
  //     controller_->reference_path_->y_.push_back(msg.poses[i].pose.position.y);
  // }

  // controller_->OnWaypointsReceived();
}

// void JackalSimulatorInterface::PartitionCallback(const lmpcc_msgs::observation_partitioning &msg)
// {
//     monitor_->MarkReceived("Partitions");

//     data_.obstacle_partitions_.clear();
//     for (size_t i = 0; i < msg.object_ids.size(); i++)
//     {
//         data_.obstacle_partitions_[msg.object_ids[i]] = msg.partitions[i]; // Map from obstacle ID -> Partition
//     }
// }

void JackalSimulatorInterface::PlotAllObstacles()
{
  RosTools::ROSPointMarker &obstacle_marker = obstacle_markers_->getNewPointMarker("CYLINDER");
  obstacle_marker.setScale(0.25, 0.25, 1.5);
  obstacle_marker.setColor(0.0, 0.0, 0.0, 0.8);
  double plot_height = 1.5;

  // Debug plot option to see the bounding boxes
  // bool plot_boxes = true;
  // RosTools::ROSPointMarker &obstacle_boxes = obstacle_markers_->getNewPointMarker("CUBE");
  // obstacle_boxes.setColor(1.0, 0.0, 0.0, 0.8);
  // obstacle_boxes.setZ(1.0);

  // Plot all obstacles
  for (auto &object : data_.dynamic_obstacles_) // obstacle_msg_.objects)
  {
    // For nicer looking figures, only plot if they will be fully in the image
    if (object.pose_.position.y > 5 || object.pose_.position.y < -5)
      continue;

    // Plot boxes shows the dimensions that carla sends us
    // if (plot_boxes)
    // {
    //     obstacle_boxes.setColor(1.0, 0.0, 0.0, 0.8);

    //     obstacle_boxes.setScale(object.shape.dimensions[0], object.shape.dimensions[1], 2.0);
    //     obstacle_boxes.addPointMarker(object.pose);
    // }
    obstacle_marker.setScale(2.0 * config_->r_VRU_, 2.0 * config_->r_VRU_, plot_height);

    // This marker is a simple cube at the exact position.
    object.pose_.position.z = 1e-3 + plot_height / 2.;
    obstacle_marker.addPointMarker(object.pose_);
    obstacle_marker.setZ(1e-3 + plot_height / 2.);
  }

  obstacle_markers_->publish();
}