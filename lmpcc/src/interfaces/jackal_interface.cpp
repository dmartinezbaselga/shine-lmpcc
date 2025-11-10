#include "interfaces/jackal_interface.h"

#include <lmpcc/lmpcc_controller.h>

#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include <std_msgs/Float64MultiArray.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>
#include <lmpcc_msgs/Control.h>

#include <derived_object_msgs/ObjectArray.h>
#include <derived_object_msgs/Object.h>

#include <lmpcc_msgs/obstacle_array.h>
#include <lmpcc_msgs/obstacle_gmm.h>
#include <lmpcc_msgs/gaussian.h>

JackalInterface::JackalInterface(ros::NodeHandle &nh, Controller *controller, predictive_configuration *config, SolverInterface *solver_interface_ptr)
    : Interface(nh, controller, config, solver_interface_ptr)
{
    LMPCC_WARN_ALWAYS("Initializing Jackal Interface...");
    monitor_.reset(new Monitor(nh)); // Monitor signals

    signal_publishers_.clear();
    signal_publishers_.emplace_back(nh, "velocity");
    signal_publishers_.emplace_back(nh, "acceleration");
    signal_publishers_.emplace_back(nh, "rotational_velocity");
    signal_publishers_.emplace_back(nh, "measured_velocity");

    velocity_sub_ = nh.subscribe(config_->robot_velocity_topic_, 1, &JackalInterface::VelocityCallback, this);
    bluetooth_sub_ = nh.subscribe("/bluetooth_teleop/joy", 1, &JackalInterface::BluetoothCallBack, this); // Deadman switch

    // Robot state
    state_sub_ = nh.subscribe("Robot_1/pose", 1, &JackalInterface::StateOptitrackCallBack, this);

    // Obstacles (OPTITRACK -> obstacle-estimator (filtering + prediction) -> LMPCC)
    for (size_t i = 0; i < config_->max_obstacles_; i++)
    {
        obstacle_subs_.push_back(
            nh.subscribe("/obstacle" + std::to_string(i + 1) + "/path_prediction", 1, &JackalInterface::OptitrackObstacleCallback, this));
        monitor_->MarkExpected("Obstacles" + std::to_string(i));
    }
    monitor_->MarkExpected("State");

    // Subscribe to waypoints
    waypoints_sub_ = nh.subscribe(config_->waypoint_topic_, 3, &Interface::RoadmapWaypointsCallback, (Interface *)this);
    goal_sub_ = nh.subscribe("/roadmap/goal", 3, &JackalInterface::GoalCallback, this);
    reverse_roadmap_pub_ = nh.advertise<std_msgs::Empty>("/roadmap/reverse", 1);

    reset_simulation_pub_ = nh.advertise<std_msgs::Empty>("/lmpcc/reset_environment", 1);

    // Add prediction subscribers for all obstacles
    for (int i = 0; i < config->max_obstacles_; i++)
        prediction_subs_.push_back(nh.subscribe("/pedestrian" + std::to_string(i + 1) + "/prediction", 1, &JackalInterface::OptitrackPredictionCallback, this));

    // (Disabled pedestrian simulator -> it should publish over optitrack channels)
    // trajectory_prediction_sub_ = nh.subscribe("/pedestrian_simulator/trajectory_predictions", 1, &JackalInterface::ObstacleTrajectoryPredictionsCallback, this);

    // Publisher for vehicle command
    command_pub_ = nh.advertise<geometry_msgs::Twist>(config_->cmd_, 1);
    last_state_time_ = ros::Time::now();
    last_reset_time_ = ros::Time::now();

    enable_output_ = false;
    last_x_ = 0.;
    last_y_ = 0.;

    data_.dynamic_obstacles_.clear();

    // Add walls!
    // data_.halfspaces_.resize(1);
    // data_.halfspaces_[0].resize(4);

    // for (int k = 0; k < solver_interface_ptr_->FORCES_N; k++)
    // {
    //     data_.halfspaces_[0][0].A_(0) = -1;
    //     data_.halfspaces_[0][0].A_(1) = 0.;
    //     data_.halfspaces_[0][0].b_ = 5.;

    //     data_.halfspaces_[0][1].A_(0) = 1;
    //     data_.halfspaces_[0][1].A_(1) = 0.;
    //     data_.halfspaces_[0][1].b_ = 5.;

    //     data_.halfspaces_[0][2].A_(0) = 0.;
    //     data_.halfspaces_[0][2].A_(1) = -1.;
    //     data_.halfspaces_[0][2].b_ = 1.;

    //     data_.halfspaces_[0][3].A_(0) = 0.;
    //     data_.halfspaces_[0][3].A_(1) = 1.;
    //     data_.halfspaces_[0][3].b_ = 8.;
    // }

    LMPCC_SUCCESS_ALWAYS("Jackal Interface Initialized");
}

void JackalInterface::ActuateNow()
{
    monitor_->PrintStatus();

    if (rotating_towards_goal_)
        RotateTowardsGoal();

    if (!enable_output_)
    {
        ROS_WARN_THROTTLE(5., "Deadman switch is disabling control");

        control_msg_.linear.x = 0;
        control_msg_.linear.y = 0;
        control_msg_.angular.z = 0;
    }

    signal_publishers_[0].Publish(solver_interface_ptr_->control_inputs_.velocity);
    signal_publishers_[1].Publish(solver_interface_ptr_->control_inputs_.acceleration);
    signal_publishers_[2].Publish(solver_interface_ptr_->control_inputs_.rot_velocity);
    signal_publishers_[3].Publish(solver_interface_ptr_->State().v());

    // Publish the command
    command_pub_.publish(control_msg_);
}

void JackalInterface::GoalCallback(const geometry_msgs::PoseStamped &msg)
{
    if (RosTools::dist(Eigen::Vector2d(msg.pose.position.x, msg.pose.position.y), data_.goal_) > 0.1)
    {
        Interface::GoalCallback(msg);
        rotating_towards_goal_ = true;
    }
}

void JackalInterface::RotateTowardsGoal()
{
    control_msg_.linear.x = 0.;

    double angle = solver_interface_ptr_->State().psi() - std::atan2(data_.goal_(1) - solver_interface_ptr_->State().y(), data_.goal_(0) - solver_interface_ptr_->State().x());
    ROS_WARN_STREAM_THROTTLE(1., "LMPCC: Rotating the robot towards the goal (angle = " << angle << ")");
    if (angle > M_PI)
        angle -= 2 * M_PI;

    if (std::abs(angle) < 0.15)
    {
        rotating_towards_goal_ = false;
        Reset(); // Reset the pedestrians
    }
    else
        control_msg_.angular.z = -1.0 * RosTools::sgn(angle);
}

void JackalInterface::Actuate()
{
    control_msg_ = geometry_msgs::Twist();

    // We use the integrated states as input

    LMPCC_WARN("==== Control Inputs ====");
    LMPCC_WARN("Velocity: " << solver_interface_ptr_->control_inputs_.velocity);
    LMPCC_WARN("Rot. Velocity: " << solver_interface_ptr_->control_inputs_.rot_velocity);
    LMPCC_WARN("========================");

    control_msg_.linear.x = solver_interface_ptr_->control_inputs_.velocity;
    control_msg_.angular.z = solver_interface_ptr_->control_inputs_.rot_velocity;
}

void JackalInterface::ActuateBrake(double deceleration)
{
    control_msg_ = geometry_msgs::Twist();

    double velocity_after_braking;
    double velocity;

    velocity = controller_->velocity_;
    velocity_after_braking = velocity - deceleration * (1.0 / ((double)config_->clock_frequency_)); // Brake with the given deceleration
    control_msg_.linear.x = std::max(velocity_after_braking, 0.);                                   // Don't drive backwards when braking

    control_msg_.angular.z = 0.0;
}

void JackalInterface::Reset()
{
    std_msgs::Empty empty_msg;
    reset_simulation_pub_.publish(empty_msg);
    controller_->objective_reached_ = false;
    // Not necessary
}

/* CALLBACKS */
void JackalInterface::ResetCallback(const std_msgs::Empty &msg)
{

    controller_->OnReset();
}

void JackalInterface::VelocityCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    LMPCC_WARN("Velocity Callback");
    double velocity = msg->twist.twist.linear.x;
    // solver_interface_ptr_->State().set_v(std::sqrt(std::pow(msg->twist.twist.linear.x, 2) + std::pow(msg->twist.twist.linear.y, 2))); // for shifting the current coordinates to the center of mass
    solver_interface_ptr_->State().set_v(velocity); // for shifting the current coordinates to the center of mass
    controller_->velocity_ = velocity;
}

void JackalInterface::OptitrackObstacleCallback(const std_msgs::Float64MultiArray &received_obstacles)
{
    LMPCC_INFO("Received obstacles from obstacle-estimator (" << received_obstacles.data.size() << " values)");
    int obstacle_id = received_obstacles.data[2];

    monitor_->MarkReceived("Obstacles" + std::to_string(obstacle_id));

    // if (data_.dynamic_obstacles_.size() > 0 && data_.dynamic_obstacles_[0].id_ == -1)
    // data_.dynamic_obstacles_.clear(); // Do not allow dummies

    // -- Convert (x, y, ID, yaw) format to an obstacle_array -- //
    Eigen::Vector2d position = Eigen::Vector2d(received_obstacles.data[0], received_obstacles.data[1]);

    DynamicObstacle *received_obstacle;
    bool obstacle_found = false;
    for (auto &obstacle : data_.dynamic_obstacles_)
    {
        if (obstacle.id_ == obstacle_id)
        {
            LMPCC_WARN("Found obstacle (ID = " << obstacle_id << ") - adding data there.");
            obstacle_found = true;
            obstacle.discs_[0].SetPosition(position);
            obstacle.pose_.position.x = position(0);
            obstacle.pose_.position.y = position(1);
            obstacle.pose_.orientation.z = received_obstacles.data[3] + M_PI_2 * config_->debug_double1_;
            received_obstacle = &obstacle;
        }
    }

    if (!obstacle_found)
    {
        LMPCC_WARN("Found NEW obstacle (ID = " << obstacle_id << ") - adding data there.");
        data_.dynamic_obstacles_.emplace_back(obstacle_id, obstacle_id, position, config_->r_VRU_);
        received_obstacle = &data_.dynamic_obstacles_.back();
    }

    // Add the predictions
    ROSTOOLS_ASSERT(received_obstacles.data.size() >= 4 * (1 + solver_interface_ptr_->FORCES_N), "Predictions should be provided for all steps.");
    received_obstacle->prediction_.gaussians.resize(1);
    received_obstacle->prediction_.gaussians[0].mean.poses.clear();
    received_obstacle->prediction_.gaussians[0].mean.poses.resize(solver_interface_ptr_->FORCES_N);
    for (int k = 0; k < solver_interface_ptr_->FORCES_N; k++) // Skip the current positions
    {
        received_obstacle->prediction_.gaussians[0].mean.poses[k].pose.position.x = received_obstacles.data[(k + 1) * 4 + 0];
        received_obstacle->prediction_.gaussians[0].mean.poses[k].pose.position.y = received_obstacles.data[(k + 1) * 4 + 1];
    }

    // handle
    // PlotObstacles
    // if (data_.dynamic_obstacles_.size() == config_->max_obstacles_)
    if (obstacle_id == first_obstacle_id_) // If the same ID is received as the trigger, run processes
    {
        PostProcessObstacles(); // This reduces/pads the obstacles to the maximum we can

        PlotObstacles(ros_markers_.get(), true, true); // This plots the obstacles that we consider
        controller_->OnObstaclesReceived();

        // data_.dynamic_obstacles_.clear(); // Do not allow dummies
    }
    else if (first_obstacle_id_ == -1)
    {
        first_obstacle_id_ = obstacle_id; // Initialize the trigger obstacle ID
        data_.dynamic_obstacles_.clear(); // Remove any dummies
    }
}

void JackalInterface::OptitrackPredictionCallback(const std_msgs::Float64MultiArray &received_obstacles)
{

    LMPCC_INFO("Received predictions from optitrack data (" << received_obstacles.data.size() << " values)");

    if (prediction_msg_.obstacles.size() > 0)
    {
        for (auto &obstacle : prediction_msg_.obstacles)
        {
            if (obstacle.id == received_obstacles.data[2])
                return;
        }
    }

    // -- Convert (x, y, ID, yaw) format to an obstacle_array -- //
    // lmpcc_msgs::obstacle_array obstacle_msg;
    prediction_msg_.obstacles.emplace_back(); // Setup a new obstacle
    prediction_msg_.obstacles.back().gaussians.emplace_back();
    prediction_msg_.obstacles.back().id = received_obstacles.data[2];

    for (size_t i = 0; i < received_obstacles.data.size(); i += 4)
    {
        // obstacle_msg.obstacles.back().gaussians.back().mean.poses.resize(solver_interface_ptr_->FORCES_N);

        geometry_msgs::Pose pose;
        pose.position.x = received_obstacles.data[i + 0];
        pose.position.y = received_obstacles.data[i + 1];

        prediction_msg_.obstacles.back().gaussians.back().mean.poses.emplace_back();
        prediction_msg_.obstacles.back().gaussians.back().mean.poses.back().pose = pose;

        // DO THE PREDICTIONS INCLUDE THE INITIAL STATE?
    }

    // std::cout << "We now have " << (int)prediction_msg_.obstacles.size() << " obstacles" << std::endl;

    // Process data only if all obstacle predictions were received
    if ((int)prediction_msg_.obstacles.size() != config_->max_obstacles_)
    {
        // std::cout << "waiting for " << config_->max_obstacles_ - (int)prediction_msg_.obstacles.size() << " obstacles" << std::endl;
        return;
    }

    // Some checks to guarantee that things are working
    assert(prediction_msg_.obstacles.size() > 0);
    assert(prediction_msg_.obstacles[0].gaussians.size() > 0);
    assert(prediction_msg_.obstacles[0].gaussians[0].mean.poses.size() > 0);

    LMPCC_INFO("\tRead " << prediction_msg_.obstacles.size() << " with horizon N = " << prediction_msg_.obstacles[0].gaussians[0].mean.poses.size());

    // We assume that all received obstacles are also predicted
    CreateObstacleList();
    PlotObstacles(ros_markers_.get(), true, true); // This plots the obstacles that we consider

    // Load predictions into the obstacles
    for (auto &obstacle_prediction : prediction_msg_.obstacles)
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

    prediction_msg_.obstacles.clear();

    controller_->OnObstaclesReceived();
}

// Create a list of dynamic obstacles
void JackalInterface::CreateObstacleList()
{
    data_.dynamic_obstacles_.clear();

    int disc_id = 0;
    for (auto &object : prediction_msg_.obstacles)
    {
        const geometry_msgs::Pose &cur_pose = object.gaussians[0].mean.poses[0].pose;
        data_.dynamic_obstacles_.emplace_back(object.id, disc_id, Eigen::Vector2d(cur_pose.position.x, cur_pose.position.y),
                                              config_->r_VRU_); // Radius = according to lmpcc config

        disc_id++;
    }
}

void JackalInterface::StateCallBack(const nav_msgs::Odometry &msg)
{
    PROFILE_FUNCTION(); // To track when the state is received
    LMPCC_WARN("State Callback");

    // Update the frame
    // config_->target_frame_ = msg.header.frame_id;
    t_state_received_ = msg.header.stamp;

    geometry_msgs::TransformStamped lookup_transform;

    tf2_ros::TransformListener tf_listener(tf_buffer_);

    lookup_transform = tf_buffer_.lookupTransform("map", "odom", ros::Time(0), ros::Duration(0.25));

    geometry_msgs::PoseStamped cur_pose;
    cur_pose.pose = msg.pose.pose;
    cur_pose.header = msg.header;
    tf2::doTransform(cur_pose, cur_pose, lookup_transform);

    // Update the states
    solver_interface_ptr_->State().set_x(cur_pose.pose.position.x); // for shifting the current coordinates to the center of mass
    solver_interface_ptr_->State().set_y(cur_pose.pose.position.y);
    solver_interface_ptr_->State().set_psi(RosTools::quaternionToAngle(cur_pose.pose.orientation));

    controller_->velocity_ = msg.twist.twist.linear.x;

    // Plots a Axis to display the path variable location
    // geometry_msgs::TransformStamped transformStamped;
    // transformStamped.header.stamp = ros::Time::now();
    // transformStamped.header.frame_id = config_->target_frame_;
    // transformStamped.child_frame_id = "robot_state";

    // transformStamped.transform.translation.x = solver_interface_ptr_->State().x();
    // transformStamped.transform.translation.y = solver_interface_ptr_->State().y();
    // transformStamped.transform.translation.z = 0.0;
    // transformStamped.transform.rotation.x = 0;
    // transformStamped.transform.rotation.y = 0;
    // transformStamped.transform.rotation.z = 0;
    // transformStamped.transform.rotation.w = 1;

    // path_pose_pub_.sendTransform(transformStamped);

    if (config_->debug_output_)
        solver_interface_ptr_->State().print();

    controller_->OnStateReceived();
}

void JackalInterface::StateOptitrackCallBack(const geometry_msgs::PoseStamped &msg)
{
    if (last_state_time_ + ros::Duration(1. / 50. /*config_->clock_frequency_*/) > msg.header.stamp)
        return;

    PROFILE_FUNCTION(); // To track when the state is received

    LMPCC_WARN("State Optitrack Callback");
    monitor_->MarkReceived("State");

    // Update the states
    solver_interface_ptr_->State().set_x(msg.pose.position.x);
    solver_interface_ptr_->State().set_y(msg.pose.position.y);
    solver_interface_ptr_->State().set_psi(RosTools::quaternionToAngle(msg.pose.orientation));
    // solver_interface_ptr_->State().set_psi(msg.pose.orientation.z);

    Eigen::Vector2d current_pos(solver_interface_ptr_->State().x(), solver_interface_ptr_->State().y());

    // Velocity estimation!
    // ros::Duration duration = msg.header.stamp - last_state_time_;
    // double time_passed_d = std::max(duration.nsec / 1.0e9, 0.0005);

    // double velocity = RosTools::dist(current_pos, Eigen::Vector2d(last_x_, last_y_)) / (time_passed_d);
    // last_x_ = solver_interface_ptr_->State().x();
    // last_y_ = solver_interface_ptr_->State().y();
    // last_state_time_ = msg.header.stamp;

    // if (((MPCC *)controller_)->controller_status_ == ControllerStatus::SUCCESS) // If feasible -> Use the velocity that we send!
    //     velocity = solver_interface_ptr_->control_inputs_.velocity;

    // solver_interface_ptr_->State().set_v(velocity);
    // controller_->velocity_ = velocity;

    if (config_->debug_output_)
        solver_interface_ptr_->State().print();

    // bool reset_condition = solver_interface_ptr_->State().y() > 3. || solver_interface_ptr_->State().y() < -3.;
    // MIN_Y = -2.5
    // MIN_X = -4.0

    // MAX_Y = 3.7
    // MAX_X = 3.5
    bool reset_condition = solver_interface_ptr_->State().y() > 3. || solver_interface_ptr_->State().y() < -2.7;
    // bool reset_condition = controller_->objective_reached_;

    if (reset_condition && last_reset_time_ + ros::Duration(15.0) < ros::Time::now()) // solver_interface_ptr_->State().y() > 5. || solver_interface_ptr_->State().y() < -1)
    {
        last_reset_time_ = ros::Time::now();
        std_msgs::Empty empty_msg;
        reverse_roadmap_pub_.publish(empty_msg);
        ResetCallback(empty_msg);
    }
    else if (reset_condition)
    {
        controller_->objective_reached_ = false;
    }

    controller_->OnStateReceived();
}

void JackalInterface::BluetoothCallBack(const sensor_msgs::Joy &msg)
{
    LMPCC_WARN("Bluetooth Callback");

    enable_output_ = msg.axes[2] < -0.9;
}

// Callback for Gaussian obstacle information (i.e., mean and covariance for all k)
// We need to associate these predictions to the obstacles here...
void JackalInterface::ObstaclePredictionsCallback(const lmpcc_msgs::obstacle_array &msg)
{
    assert(false);
    // if (config_->use_trajectory_sampling_)
    //     return;

    // LMPCC_INFO("JackalSimulatorInterface::ObstaclePredictionsCallback received " << msg.obstacles.size() << " obstacle predictions");
    // if (obstacle_msg_.objects.size() == 0 || (int)obstacle_msg_.objects[0].id == -1)
    // {
    //     LMPCC_WARN("Predictions received when obstacle positions are unknown, not loading.");
    //     return;
    // }

    // prediction_msg_ = msg; // Save the data

    // // We assume that all received obstacles are also predicted
    // CreateObstacleList();

    // // Load predictions into the obstacles
    // for (auto &obstacle_prediction : msg.obstacles)
    // {
    //     for (auto &obstacle : data_.dynamic_obstacles_)
    //     {
    //         // Association based on id
    //         if (obstacle_prediction.id == obstacle.id_)
    //         {
    //             obstacle.LoadPredictions(obstacle_prediction);
    //             std::cout << "set prediction for obstacle " << obstacle.id_ << std::endl;
    //             break;
    //         }
    //     }
    // }
}

void JackalInterface::ObstacleTrajectoryPredictionsCallback(const lmpcc_msgs::obstacle_array &msg)
{

    // LMPCC_INFO("JackalInterface::ObstacleTrajectoryPredictionsCallback received " << msg.obstacles.size() << " obstacle predictions");
    // if (obstacle_msg_.objects.size() == 0 || (int)obstacle_msg_.objects[0].id == -1)
    // {
    //     LMPCC_WARN("Predictions received when obstacle positions are unknown, not loading.");
    //     return;
    // }
    // prediction_msg_ = msg; // Save the data

    // // We assume that all received obstacles are also predicted
    // CreateObstacleList();

    // // Load predictions into the obstacles
    // for (auto &obstacle_prediction : msg.obstacles)
    // {
    //     for (auto &obstacle : data_.dynamic_obstacles_)
    //     {
    //         // Association based on id
    //         if (obstacle_prediction.id == obstacle.id_)
    //         {
    //             obstacle.LoadPredictions(obstacle_prediction);
    //             break;
    //         }
    //     }
    // }

    // controller_->OnObstaclesReceived();
}
