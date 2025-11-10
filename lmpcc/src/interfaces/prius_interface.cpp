#include "interfaces/prius_interface.h"
#include "lmpcc_controller.h"

PriusInterface::PriusInterface(ros::NodeHandle &nh, MPCC *controller, predictive_configuration *config, SolverInterface *solver_interface_ptr)
    : Interface(nh, controller, config, solver_interface_ptr)
{
    LMPCC_INFO("Initializing Prius Interface.");

    obstacle_markers_.reset(new RosTools::ROSMarkerPublisher(nh, config_->obstacles_received_topic_.c_str(), config_->target_frame_, 20)); // 3500)); // was 1800

    monitor_.reset(new Monitor(nh)); // Monitor incoming callbacks for easy debugging
    monitor_->MarkExpected("Pose");
    monitor_->MarkExpected("Steering Angle");
    monitor_->MarkExpected("Predictions");
    monitor_->MarkExpected("Waypoints", true);

    // Subscribers for sensor data
    state_sub_ = nh.subscribe(config_->robot_state_topic_, 1, &PriusInterface::StateCallBack, this);
    steering_sub_ = nh.subscribe(config_->steering_state_topic_, 1, &PriusInterface::SteeringAngleCallback, this);
    acceleration_sub_ = nh.subscribe(config_->acceleration_state_topic_, 1, &PriusInterface::AccelerationCallback, this);

    // Subscribers for high level data
    obstacle_prediction_sub_ = nh.subscribe(config_->obstacle_prediction_topic_, 1, &PriusInterface::ObstacleCallBack, this);
    reference_path_sub_ = nh.subscribe(config_->waypoint_topic_, 1, &PriusInterface::WaypointsCallback, this);

    // Publisher for vehicle command
    command_pub_ = nh.advertise<lmpcc_msgs::Control>(config_->cmd_, 1);
    vehicle_speed_pub_ = nh.advertise<geometry_msgs::Twist>("lmpcc/vehicle_speed", 1); // Used with simulated pedestrians

    // For resetting simulated pedestrians
    reset_simulation_pub_ = nh.advertise<std_msgs::Empty>(config_->reset_environment_topic_.c_str(), 1);

    N_pub_ = nh.advertise<std_msgs::Float64>("pedestrian_simulator/N", 1);
    dt_pub_ = nh.advertise<std_msgs::Float64>("pedestrian_simulator/dt", 1);
    hz_pub_ = nh.advertise<std_msgs::Float64>("pedestrian_simulator/Hz", 1);

    // Initialize the obstacles variable
    obstacles_.lmpcc_obstacles.resize(config_->max_obstacles_);
    for (int obst_it = 0; obst_it < config_->max_obstacles_; obst_it++)
    {
        obstacles_.lmpcc_obstacles[obst_it].trajectory.poses.resize(solver_interface_ptr_->FORCES_N);
        obstacles_.lmpcc_obstacles[obst_it].major_semiaxis.resize(solver_interface_ptr_->FORCES_N);
        obstacles_.lmpcc_obstacles[obst_it].minor_semiaxis.resize(solver_interface_ptr_->FORCES_N);
        obstacles_.lmpcc_obstacles[obst_it].id = obst_it;
        for (unsigned int t = 0; t < solver_interface_ptr_->FORCES_N; t++)
        {
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[t].pose.position.x = 50; // Lowered...
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[t].pose.position.y = 50;
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[t].pose.orientation.z = 0;
            obstacles_.lmpcc_obstacles[obst_it].major_semiaxis[t] = 0.1;
            obstacles_.lmpcc_obstacles[obst_it].minor_semiaxis[t] = 0.1;
        }
    }

    plot_throttle_pub_ = nh.advertise<std_msgs::Float32>("/lmpcc/throttle", 1); /* Publish the sample size when it is incorrect */

    // DEBUG ONLY
    // DebugInsertFakeObstacles(); // DISABLED
    reference_received_ = false;

    LMPCC_SUCCESS("Prius Interface Initialized");
}

void PriusInterface::Actuate()
{

    monitor_->PrintStatus();

    lmpcc_msgs::Control control_msg_;

    // Initialize the header
    control_msg_.header.frame_id = config_->target_frame_;
    control_msg_.header.stamp = ros::Time::now();

    // We use the integrated states as input
    control_msg_.throttle = solver_interface_ptr_->a();

    control_msg_.steer = solver_interface_ptr_->control_inputs_.steering;

    control_msg_.brake = 0.0;

    // From old code!
    // vehicle_cmd_.throttle = forces_output.x02[9]; // Note: input is jerk, acceleration at 9th state
    // vehicle_cmd_.steer = forces_output.x02[7]; // Note: input is deltasteering, steering at 7th state

    // Publish the command
    command_pub_.publish(control_msg_);

    LMPCC_WARN_STREAM("\n========== COMMAND ==========\n"
                      << "delta\t=\t" << control_msg_.steer << "\n"
                      << "a\t=\t" << control_msg_.throttle << "\n"
                      << "============================\n");

    std_msgs::Float32 throttle_plot_msg;
    throttle_plot_msg.data = solver_interface_ptr_->a();
    plot_throttle_pub_.publish(throttle_plot_msg);

    // To motion compensate the pedestrians
    geometry_msgs::Twist vehicle_speed_msg_;
    vehicle_speed_msg_.linear.x = solver_interface_ptr_->v(0) * std::cos(solver_interface_ptr_->State().psi());
    vehicle_speed_msg_.linear.y = solver_interface_ptr_->v(0) * std::sin(solver_interface_ptr_->State().psi());
    vehicle_speed_msg_.angular.z = solver_interface_ptr_->w();
    vehicle_speed_pub_.publish(vehicle_speed_msg_);
}

void PriusInterface::ActuateBrake(double deceleration)
{
    monitor_->PrintStatus();

    lmpcc_msgs::Control control_msg_;
    deceleration = std::abs(deceleration); /** Ensure positive deceleration for safety */

    // Initialize the header
    control_msg_.header.frame_id = config_->target_frame_;
    control_msg_.header.stamp = ros::Time::now();

    // We use the integrated states as input
    control_msg_.throttle = -deceleration; // Decelerate
    control_msg_.steer = 0.0;
    control_msg_.brake = 0.0; // Braking is disabled

    // Publish the command
    command_pub_.publish(control_msg_);
}

void PriusInterface::Reset()
{
    reset_simulation_pub_.publish(std_msgs::Empty()); // Reset simulated pedestrians

    // Send pedestrian simulator settings
    std_msgs::Float64 N_msg, dt_msg, hz_msg;
    N_msg.data = solver_interface_ptr_->FORCES_N;
    N_pub_.publish(N_msg);

    dt_msg.data = solver_interface_ptr_->DT;
    dt_pub_.publish(dt_msg);

    hz_msg.data = config_->clock_frequency_;
    hz_pub_.publish(hz_msg);
}

/* CALLBACKS */

void PriusInterface::ObstacleCallBack(const lmpcc_msgs::obstacle_array &msg)
{
    // Is also used to read the predictions!
    LMPCC_INFO_STREAM("PriusInterface::ObstacleCallback (with " << msg.obstacles.size() << " obstacles)");
    monitor_->MarkReceived("Predictions");

    obstacle_msg_ = msg;

    PreProcessObstacles();  // Convert received obstacles to the format used by the controller
    PlotAllObstacles();     // Plot all obstacles
    PostProcessObstacles(); // Ensure that we fill the rest of the obstacles with dummies (or sort the obstacles)

    LMPCC_INFO_STREAM("Prius Interface: Converted " << data_.dynamic_obstacles_.size() << " obstacles to LMPCC format");

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

    controller_->OnObstaclesReceived(); // Process in the modules if necessary
}

// Create a list of dynamic obstacles
void PriusInterface::PreProcessObstacles()
{
    data_.dynamic_obstacles_.clear();

    int disc_id = 0;
    for (auto &obstacle : obstacle_msg_.obstacles)
    {
        // Pedestrians as a single disc
        data_.dynamic_obstacles_.emplace_back(
            obstacle.id,
            disc_id,
            Eigen::Vector2d(obstacle.pose.position.x, obstacle.pose.position.y),
            config_->r_VRU_);

        data_.dynamic_obstacles_.back().pose_ = obstacle.pose;

        // data_.dynamic_obstacles_.back().PredictConstantVelocity(object.twist, solver_interface_ptr_->DT, solver_interface_ptr_->FORCES_N, 1.0 / 9., 1.0);

        disc_id++;
    }
}

void PriusInterface::StateCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    LMPCC_INFO("State Callback");
    monitor_->MarkReceived("Pose");

    // Update the frame
    config_->target_frame_ = msg->header.frame_id;

    // Update the states
    solver_interface_ptr_->State().set_psi(RosTools::quaternionToAngle(msg->pose.pose));
    solver_interface_ptr_->State().set_x(msg->pose.pose.position.x + 1.577 * cos(solver_interface_ptr_->State().psi())); // for shifting the current coordinates to the center of mass
    solver_interface_ptr_->State().set_y(msg->pose.pose.position.y + 1.577 * sin(solver_interface_ptr_->State().psi()));

    // Update the velocity (if the vehicle is in debug mode, allow for the velocity to be simulated)
    solver_interface_ptr_->State().set_v(std::sqrt(std::pow(msg->twist.twist.linear.x, 2) + std::pow(msg->twist.twist.linear.y, 2)) + config_->debug_velocity_);

    if (config_->debug_output_)
        solver_interface_ptr_->State().print();

    controller_->OnStateReceived();
}

// Callback for ax and ay
void PriusInterface::AccelerationCallback(const geometry_msgs::AccelWithCovarianceStamped &msg)
{
    LMPCC_INFO("Acceleration Callback (DISABLED FOR MODEL WITH ACCELERATION INPUT)");

    // solver_interface_ptr_->State().set_a(msg.accel.accel.linear.x);
    // solver_interface_ptr_->State().set_ay(msg.accel.accel.linear.y);
}

/* Callback for the steering joint feedback */
void PriusInterface::SteeringAngleCallback(const sensor_msgs::JointState &msg)
{
    LMPCC_INFO("Steering Callback");
    monitor_->MarkReceived("Steering Angle");

    // The steering angle is saved in the first position
    // if (!delayed_)
    solver_interface_ptr_->State().set_delta(msg.position[0]);
    // else
    // solver_interface_ptr_->State().set_delta_in(msg.position[0]);
}

void PriusInterface::PlotAllObstacles()
{
    RosTools::ROSPointMarker &obstacle_marker = obstacle_markers_->getNewPointMarker("CYLINDER");
    obstacle_marker.setScale(0.25, 0.25, 1.5);
    obstacle_marker.setColor(0.0, 0.0, 0.0, 0.8);
    double plot_height = 1.5;

    // Plot all obstacles
    for (auto &object : data_.dynamic_obstacles_) // obstacle_msg_.objects)
    {

        obstacle_marker.setScale(2.0 * config_->r_VRU_, 2.0 * config_->r_VRU_, plot_height);

        // This marker is a cylinder at the exact position.
        object.pose_.position.z = 1e-3 + plot_height / 2.;
        obstacle_marker.addPointMarker(object.pose_);
        obstacle_marker.setZ(1e-3 + plot_height / 2.);
    }

    obstacle_markers_->publish();
}

void PriusInterface::WaypointsCallback(const nav_msgs::Path &msg)
{
    LMPCC_INFO_STREAM("Received " << std::floor(msg.poses.size()) << " Waypoints");
    monitor_->MarkReceived("Waypoints");

    // If new waypoints are close to the old don't do anything
    bool waypoints_are_the_same = data_.path_.Get().x_.size() == msg.poses.size();

    if (waypoints_are_the_same) // Check if they really are the same
    {

        for (size_t i = 0; i < msg.poses.size(); i++)
        {
            if (std::sqrt(std::pow(msg.poses[i].pose.position.x - data_.path_.Get().x_[i], 2) +
                          std::pow(msg.poses[i].pose.position.y - data_.path_.Get().y_[i], 2)) > 1.0)
            {
                waypoints_are_the_same = false;
                break;
            }
        }
    }

    // Do not update the reference path if the received path is the same (to reduce computation times)
    if (waypoints_are_the_same)
        return;

    data_.path_.Get().Clear(); // Clear the path

    for (size_t i = 0; i < msg.poses.size(); i++)
        data_.path_.Get().AddPose(msg.poses[i].pose); // Add waypoints

    controller_->OnWaypointsReceived(); // Update the reference path
}

void PriusInterface::DebugInsertFakeObstacles()
{
    LMPCC_WARN_ALWAYS("Inserting FAKE obstacles");
    for (int i = 0; i < config_->max_obstacles_; i++)
    {

        geometry_msgs::Pose fake_pose;
        fake_pose.position.x = 7.5;
        fake_pose.position.y = -2.;
        data_.dynamic_obstacles_.emplace_back(
            i, /** Association ID of the obstacle */
            i, /** Index of the first disc of this obstacle */
            Eigen::Vector2d(fake_pose.position.x, fake_pose.position.y),
            config_->r_VRU_ /** Obstacle radius */
        );
        data_.dynamic_obstacles_.back().pose_ = fake_pose;

        geometry_msgs::Twist fake_twist;
        fake_twist.linear.x = 0.;
        fake_twist.linear.y = 0.;
        fake_twist.linear.z = 0.;

        // Insert constant velocity predictions
        data_.dynamic_obstacles_.back().PredictConstantVelocity(fake_twist, solver_interface_ptr_->DT, solver_interface_ptr_->FORCES_N);
    }
}
