#include "interfaces/jackalsocnavbench_interface.h"

#include <lmpcc/lmpcc_controller.h> 
#include <ros_tools/helpers.h> 


static inline double normalize_angle(double angle)
  {
    const double result = fmod(angle + M_PI, 2.0*M_PI);
    if(result <= 0.0) return result + M_PI;
    return result - M_PI;
}

JackalSocnavbenchInterface::JackalSocnavbenchInterface(ros::NodeHandle &nh, MPCC *controller, predictive_configuration *config, SolverInterface *solver_interface_ptr)
    : Interface(nh, controller, config, solver_interface_ptr)
{
  ROS_WARN("Initializing Jackal Socnavbench Interface...");
  obstacle_markers_.reset(new RosTools::ROSMarkerPublisher(nh, "lmpcc/received_obstacles", config_->target_frame_, 20)); // 3500)); // was 1800
  road_markers_.reset(new RosTools::ROSMarkerPublisher(nh, "lmpcc/simulation/road", config_->target_frame_, 20));        // 3500)); //
                                                                                                                         // was 1800

  monitor_.reset(new Monitor(nh)); // Monitor signals

  monitor_->MarkExpected("State");
  monitor_->MarkExpected("Predictions");
  monitor_->MarkExpected("Partitions");

  ros::service::waitForService("/select_guidance");
  shine_srv_ = nh2.advertiseService("/solve_shine", &JackalSocnavbenchInterface::handleShine, this);
  time_pub_ = nh.advertise<std_msgs::Time>("/time", 1);

  LMPCC_WARN("Jackal Interface Initialized");
}

void JackalSocnavbenchInterface::ActuateNow()
{
  // Publish the command
  // command_pub_.publish(control_msg_);

  // // Publish signals for visuals
  // signal_publishers_[0].Publish(solver_interface_ptr_->control_inputs_.acceleration);
  // signal_publishers_[1].Publish(solver_interface_ptr_->control_inputs_.velocity);
  // signal_publishers_[2].Publish(solver_interface_ptr_->control_inputs_.rot_velocity);

  control_msg_ = geometry_msgs::Twist();

  control_msg_.linear.x = solver_interface_ptr_->control_inputs_.velocity;
  control_msg_.angular.z = solver_interface_ptr_->control_inputs_.rot_velocity;
}

bool JackalSocnavbenchInterface::rotateToGoal(double x, double y, double theta, double goal_x, double goal_y, double& v, double& w)
{
    double goal_angle = std::atan2(goal_y - y, goal_x - x);

    double angle_diff = goal_angle - theta;

    angle_diff = normalize_angle(angle_diff);

    if (std::abs(angle_diff) > M_PI / 3.)
    {
        v = 0.0;
        w = 1.1 * RosTools::sgn(angle_diff);
        return true;
    }
    else
    {
        return false;
    }
}

bool JackalSocnavbenchInterface::handleShine(lmpcc_msgs::SolveMPC::Request& req, lmpcc_msgs::SolveMPC::Response& res){
  // if (config_->debug_output_)
  //   std::cout << std::endl;
  req.input.theta = normalize_angle(req.input.theta);
  if (!rotateToGoal(req.input.x, req.input.y, req.input.theta, req.input.goal_x, req.input.goal_y, res.v, res.w)){
    LMPCC_INFO("============ UPDATE DATA =======");
    updateData(req);
    LMPCC_INFO("============ START OF LOOP [ =======");
    controller_->RunOnDemand();
    

    // // if (config_->debug_output_)
    // //   std::cout << std::endl;

    ActuateNow();
    
    res.v = control_msg_.linear.x;
    res.w = control_msg_.angular.z;
    LMPCC_INFO("============ END OF SHINE ===============");
  }
  return true;
}

void JackalSocnavbenchInterface::Actuate()
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

void JackalSocnavbenchInterface::ActuateBrake(double deceleration)
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

void JackalSocnavbenchInterface::Reset()
{
    // controller_->OnReset();
}

void JackalSocnavbenchInterface::updateData(const lmpcc_msgs::SolveMPC::Request& req){
  std_msgs::Time time_msg;
  time_msg.data = req.input.obstacles.header.stamp;
  time_pub_.publish(time_msg);
  ros::spinOnce();
  LMPCC_INFO("Marking state");
  monitor_->MarkReceived("State");
  LMPCC_INFO("State marked");
  // Update the frame
  config_->target_frame_ = "map";

  // Update the states
  solver_interface_ptr_->State().set_x(req.input.x); // for shifting the current coordinates to the center of
                                                              // mass
  solver_interface_ptr_->State().set_y(req.input.y);
  solver_interface_ptr_->State().set_psi(req.input.theta);
  controller_->velocity_ = req.input.v;
  data_.velocity_ = req.input.v;
  solver_interface_ptr_->State().set_v(req.input.v); // When v is a state
  LMPCC_INFO("State saved");
  data_.state_received_time_ = req.input.obstacles.header.stamp;
  LMPCC_INFO("Time saved");


  // Plots a Axis to display the path variable location

  if ((t_last_tf_ - req.input.obstacles.header.stamp).toSec() > 0.01)
  {
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = req.input.obstacles.header.stamp;
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

  LMPCC_INFO("JackalSocnavbenchInterface::ObstaclePredictionsCallback received " << req.input.obstacles.obstacles.size() << " obstacle predictions");
  monitor_->MarkReceived("Predictions");

  // Load all obstacles into our format
  data_.dynamic_obstacles_.clear();

  int disc_counter = 0; /** Track the number of obstacle discs */

  // Note: We loop until we have all the DISCS that we can handle
  for (auto &obstacle : req.input.obstacles.obstacles)
  {
    // Pedestrians as a single disc
    data_.dynamic_obstacles_.emplace_back(obstacle.id, disc_counter, Eigen::Vector2d(obstacle.pose.position.x, obstacle.pose.position.y), config_->r_VRU_);
    // std::sqrt(//std::pow(object.shape.dimensions[0] / 2., 2.) + std::pow(object.shape.dimensions[1] / 2., 2.)));

    data_.dynamic_obstacles_.back().pose_ = obstacle.pose;

    disc_counter++;
  }

  PlotAllObstacles();

  PostProcessObstacles();

  // Load predictions into the obstacles
  for (auto &obstacle_prediction : req.input.obstacles.obstacles)
  {
    for (auto &obstacle : data_.dynamic_obstacles_)
    {
      // Association based on id
      if (obstacle_prediction.id == obstacle.id_)
      { 
        lmpcc_msgs::obstacle_gmm obstacle_msg;
        obstacle_msg.id = obstacle_prediction.id;
        obstacle_msg.pose.position.x = obstacle_prediction.pose.position.x;
        obstacle_msg.pose.position.y = obstacle_prediction.pose.position.y;
        nav_msgs::Path path_msg;
        for (int i_obs = 0; i_obs < N; i_obs++){
          geometry_msgs::PoseStamped pose_msg;
          // Obstacle orientation is in obstacle_prediction.pose.position.z and velocity is in obstacle_prediction.pose.orientation.x
          pose_msg.pose.position.x = obstacle_prediction.pose.position.x + cos(obstacle_prediction.pose.position.z) * obstacle_prediction.pose.orientation.x * dt * (i_obs+1);
          pose_msg.pose.position.y = obstacle_prediction.pose.position.y + sin(obstacle_prediction.pose.position.z) * obstacle_prediction.pose.orientation.x * dt * (i_obs+1);
          path_msg.poses.push_back(pose_msg);
        }
        lmpcc_msgs::gaussian gaussian_msg;
        gaussian_msg.mean=path_msg;
        obstacle_msg.gaussians.push_back(gaussian_msg); 
        obstacle.LoadPredictions(obstacle_msg);
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

  data_.goal_(0) = req.input.goal_x;
  data_.goal_(1) = req.input.goal_y;

  controller_->OnOtherDataReceived("Goal");
}

void JackalSocnavbenchInterface::PlotRoad()
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


void JackalSocnavbenchInterface::PlotAllObstacles()
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