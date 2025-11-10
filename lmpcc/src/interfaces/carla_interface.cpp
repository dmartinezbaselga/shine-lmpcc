#include "interfaces/carla_interface.h"
#include "lmpcc_controller.h"

CarlaInterface::CarlaInterface(ros::NodeHandle &nh, MPCC *controller, predictive_configuration *config, SolverInterface *solver_interface_ptr)
    : Interface(nh, controller, config, solver_interface_ptr)
{
    LMPCC_WARN("Initializing Carla Interface.");

    solver_interface_ptr_ = solver_interface_ptr;

    // ros_markers_.reset(new RosTools::ROSMarkerPublisher(nh, "lmpcc/straight_road", config_->target_frame_, 5));//3500)); // was 1800
    ros_all_obstacle_markers_.reset(new RosTools::ROSMarkerPublisher(nh, config_->obstacles_received_topic_.c_str(), config_->target_frame_, 100));       // 3500)); // was 1800
    ros_predicted_obstacle_markers_.reset(new RosTools::ROSMarkerPublisher(nh, config_->obstacles_predicted_topic_.c_str(), config_->target_frame_, 20)); // 3500)); // was 1800

    monitor_.reset(new Monitor(nh));

    // Subscribers for sensor data
    state_sub_ = nh.subscribe(config_->robot_state_topic_, 2, &CarlaInterface::StateCallBack, this);

    // steering_sub_ = nh.subscribe(config_->steering_state_topic_, 1, &CarlaInterface::SteeringAngleCallback, this);
    acceleration_sub_ = nh.subscribe(config_->acceleration_state_topic_, 1, &CarlaInterface::AccelerationCallback, this);

    // PARTITION
    obstacle_sub_ = nh.subscribe(config_->obs_state_topic_, 1, &CarlaInterface::ObstacleCallBack, this);

    // GENERAL
    obstacle_prediction_sub_ = nh.subscribe(config_->obstacle_prediction_topic_.c_str(), 1, &CarlaInterface::ObstacleTrajectoryPredictionsCallback, this);

    // Subscribe to waypoints
    waypoints_sub_ = nh.subscribe(config_->waypoint_topic_, 1, &Interface::RoadmapWaypointsCallback, (Interface *)this);

    // Vehicle Info
    vehicle_info_sub_ = nh.subscribe("/carla/ego_vehicle/vehicle_info", 1, &CarlaInterface::VehicleInfoCallback, this);

    // Subscribe to initial pose estimate in rviz (set this topic in the rviz config!)
    reset_pose_sub_ = nh.subscribe("/lmpcc/initialpose", 1, &CarlaInterface::ResetCallback, this);

    // Publisher to mark the reset, can be used by other modules to reset when the LMPCC resets
    reset_pub_ = nh.advertise<std_msgs::Empty>("/lmpcc/reset_environment", 1);

    // Debugging callback for Carla
    carla_status_sub_ = nh.subscribe("/carla/status", 1, &CarlaInterface::carlaStatusCallback, this);
    goal_reached_sub = nh.subscribe("/carla/ego_vehicle/goal/", 1, &CarlaInterface::carlaGoalPositionCallback, this);
    // Subscriber for receiving halfspaces (for example for the road limits)
    halfspace_sub_ = nh.subscribe("/lmpcc/road_limits", 1, &CarlaInterface::halfspaceCallback, this);

    // Publisher for vehicle command
    command_pub_ = nh.advertise<ackermann_msgs::AckermannDrive>(config_->cmd_, 1); // Use ackermann control

    obstacles_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/obstacles", 1);

    sample_size_pub_ = nh.advertise<std_msgs::Int32>("/lmpcc/sample_size", 1); /* Publish the sample size when it is incorrect */

    // Enable / disable to receive info on the actual vehicle commands after the ackermann node!
    ackermann_info_sub_ = nh.subscribe("/carla/ego_vehicle/ackermann_control/control_info", 1, &CarlaInterface::AckermannCallback, this);

    if (config_->use_real_samples_)
        partition_sub_ = nh.subscribe("/lmpcc/observation_partitions", 1, &Interface::PartitionCallback, (Interface *)this);

    // To add for publishing the goal
    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>(config_->navigation_goal_topic_, 1);
    goal_set_ = false; // Publish when the waypoint publisher is online
    goal_location_ = Eigen::Vector2d(324, -2);
    collision_detected_ = false;

    monitor_->MarkExpected("Pose");

    if (!config_->use_real_samples_) // In this case we locally retrieve samples
        monitor_->MarkExpected("Predictions");

    monitor_->MarkExpected("Waypoints", true);

    /* For JSK visuals */
    signal_publishers_.emplace_back(nh, "throttle");
    signal_publishers_.emplace_back(nh, "vx");
    signal_publishers_.emplace_back(nh, "delta");
    signal_publishers_.emplace_back(nh, "omega");

    last_transform_time_ = ros::Time(0);

    last_velocity_ = 0.0;
    received_external_prediction_ = false;
    collision_detected_ = false;

    reset_carla_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(config_->reset_environment_topic_, 1);
    N_pub_ = nh.advertise<std_msgs::Float64>("pedestrian_simulator/N", 1);
    dt_pub_ = nh.advertise<std_msgs::Float64>("pedestrian_simulator/dt", 1);
    hz_pub_ = nh.advertise<std_msgs::Float64>("pedestrian_simulator/Hz", 1);

    sample_size_pub_ = nh.advertise<std_msgs::Int32>("/lmpcc/sample_size", 1);

    // Dummy obstacle definition
    dummy_obstacle_.header.frame_id = "map";
    dummy_obstacle_.header.stamp = ros::Time::now();
    dummy_obstacle_.id = -1;
    dummy_obstacle_.classification = derived_object_msgs::Object::CLASSIFICATION_PEDESTRIAN;

    dummy_obstacle_.shape.dimensions.resize(2);
    dummy_obstacle_.shape.dimensions[0] = 0.01;
    dummy_obstacle_.shape.dimensions[1] = 0.01;

    dummy_obstacle_.twist.linear.x = 0.;
    dummy_obstacle_.twist.linear.y = 0.;
    dummy_obstacle_.twist.angular.x = 0.;
    dummy_obstacle_.twist.angular.y = 0.;
    dummy_obstacle_.twist.angular.z = 0.;

    dummy_obstacle_.pose.orientation.w = 1.;
    dummy_obstacle_.pose.orientation.x = 0;
    dummy_obstacle_.pose.orientation.y = 0;
    dummy_obstacle_.pose.orientation.z = 0;

    LMPCC_WARN_ALWAYS("Carla Interface Initialized");
}

// Callback to get Ego vehicle information
void CarlaInterface::VehicleInfoCallback(const carla_msgs::CarlaEgoVehicleInfo &msg)
{
    LMPCC_INFO_STREAM("CarlaInterface::VehicleInfoCallback\n"
                      << msg);
    ego_vehicle_info_ = msg;
}

// Callback for steering angle (carla ego_vehicle status or next control input), can add gnss via navsat launch in robot_localization
// Maybe the virtual wall is related to the spline index limit. Otherwise disable obstacles fully to be sure.

void CarlaInterface::Actuate()
{
    monitor_->PrintStatus();

    command_msg_.steering_angle = solver_interface_ptr_->control_inputs_.steering;
    command_msg_.steering_angle_velocity = solver_interface_ptr_->control_inputs_.rot_velocity;

    // Throttle (integrated to the next control time step)
    command_msg_.speed = solver_interface_ptr_->control_inputs_.velocity; // solver_interface_ptr_->v();
    command_msg_.acceleration = solver_interface_ptr_->control_inputs_.acceleration;
    command_pub_.publish(command_msg_);

    signal_publishers_[0].Publish(command_msg_.acceleration);
    signal_publishers_[1].Publish(command_msg_.speed);
    signal_publishers_[2].Publish(command_msg_.steering_angle);
    signal_publishers_[3].Publish(command_msg_.steering_angle_velocity);

    // There is definitely an issue here, we are simulating 200 ms per step which is delta(1), but the new input is actuated after 100 ms, i.e., there is a mismatch!
    // The integration however doesn't work for some reason
    // if (!delayed_)
    // solver_interface_ptr_->State().set_delta(solver_interface_ptr_->delta(1)); // This is an issue to solve (carla doesn't have an API for this variable it seems)
    // else
    // solver_interface_ptr_->State().set_delta_in(solver_interface_ptr_->delta_in(1));
    solver_interface_ptr_->State().set_delta(solver_interface_ptr_->delta(1));

    LMPCC_WARN("Sending Command to Vehicle");
    LMPCC_WARN_STREAM("\n========== COMMAND ==========\n"
                      << "delta\t=\t" << command_msg_.steering_angle << "\n"
                      << "v\t=\t" << command_msg_.speed << "\n"
                      << "a\t=\t" << command_msg_.acceleration << "\n"
                      << "============================\n");
}

void CarlaInterface::ActuateBrake(double deceleration)
{
    monitor_->PrintStatus();

    deceleration = std::abs(deceleration);

    LMPCC_INFO_STREAM("Actuating the Brake: " << deceleration);

    // Ackermann
    // command_msg_.steering_angle = solver_interface_ptr_->State().delta_in();
    command_msg_.steering_angle = solver_interface_ptr_->State().delta();
    command_msg_.steering_angle_velocity = 0.0;

    // Go to zero speed with deceleration
    double dt = 1.0 / ((double)config_->clock_frequency_);
    command_msg_.speed = solver_interface_ptr_->State().v() - deceleration * dt;
    command_msg_.acceleration = -deceleration;

    command_pub_.publish(command_msg_);
}

// Reset the simulation (internal)
void CarlaInterface::Reset(const geometry_msgs::Pose &position_after_reset)
{
    geometry_msgs::PoseWithCovarianceStamped reset_msg;
    reset_msg.pose.pose = position_after_reset;

    for (int j = 0; j < 10; j++)
    {
        ActuateBrake(0.0);
        ros::Duration(1.0 / config_->clock_frequency_).sleep();
    }
    reset_carla_pub_.publish(reset_msg);

    std_msgs::Float64 N_msg, dt_msg, hz_msg;
    N_msg.data = solver_interface_ptr_->FORCES_N;
    N_pub_.publish(N_msg);

    dt_msg.data = solver_interface_ptr_->DT;
    dt_pub_.publish(dt_msg);

    hz_msg.data = config_->clock_frequency_;
    hz_pub_.publish(hz_msg);

    std_msgs::Empty empty_reset_msg; // Communicate the reset outwards
    reset_pub_.publish(empty_reset_msg);

    ros::Duration(0.5).sleep();
}

void CarlaInterface::Reset()
{
    Reset(reset_pose_);
}

/* CALLBACKS */
// Callback for resetting the LMPCC (external)
void CarlaInterface::ResetCallback(const geometry_msgs::PoseWithCovarianceStamped &initial_pose)
{
    // Set reset position here.
    reset_pose_ = initial_pose.pose.pose;

    controller_->OnReset();
}

void CarlaInterface::ObstacleCallBack(const derived_object_msgs::ObjectArray &received_obstacles)
{
    LMPCC_INFO_STREAM("Carla Interface: ObstacleCallBack received " << received_obstacles.objects.size() << " obstacles.");

    if (ego_vehicle_info_.id == 0)
    {
        LMPCC_INFO("\tDid not yet receive the ego-vehicle data. Returning.");
        return;
    }

    // std::cout << "Vehicle cog: " << ego_vehicle_info_.center_of_mass.x << ", " << ego_vehicle_info_.center_of_mass.y << std::endl;

    // Load all obstacles into our format
    data_.dynamic_obstacles_.clear();

    int disc_counter = 0; /** Track the number of obstacle discs */

    // Note: We loop until we have all the DISCS that we can handle
    for (auto &object : received_obstacles.objects)
    {
        if (object.id == ego_vehicle_info_.id) // The ego-vehicle is not an obstacle
            continue;

        // Pedestrians as a single disc
        data_.dynamic_obstacles_.emplace_back(
            object.id,
            disc_counter,
            Eigen::Vector2d(object.pose.position.x, object.pose.position.y),
            config_->r_VRU_);

        disc_counter++;

        data_.dynamic_obstacles_.back().pose_ = object.pose;
        data_.dynamic_obstacles_.back().PredictConstantVelocity(object.twist, solver_interface_ptr_->DT, solver_interface_ptr_->FORCES_N);
    }

    LMPCC_INFO_STREAM("Carla Interface: Converted " << data_.dynamic_obstacles_.size() << " obstacles to LMPCC format");

    PostProcessObstacles(&CarlaInterface::ObstacleRejectFunction);    // This reduces the obstacles to the maximum we can handle
    PlotObstacles(ros_predicted_obstacle_markers_.get(), true, true); // This plots the obstacles that we consider

    controller_->OnObstaclesReceived();
}

void CarlaInterface::ObstacleTrajectoryPredictionsCallback(const lmpcc_msgs::obstacle_array &msg)
{

    LMPCC_INFO("CarlaInterface::ObstacleTrajectoryPredictionsCallback received " << msg.obstacles.size() << " obstacle predictions");
    monitor_->MarkReceived("Predictions");

    prediction_msg_ = msg; // Save the data

    // Load all obstacles into our format
    data_.dynamic_obstacles_.clear();

    int disc_counter = 0; /** Track the number of obstacle discs */

    // Note: We loop until we have all the DISCS that we can handle
    for (auto &obstacle : msg.obstacles)
    {
        // Pedestrians as a single disc
        data_.dynamic_obstacles_.emplace_back(
            obstacle.id,
            disc_counter,
            Eigen::Vector2d(obstacle.pose.position.x, obstacle.pose.position.y),
            config_->r_VRU_);
        // std::sqrt(//std::pow(object.shape.dimensions[0] / 2., 2.) + std::pow(object.shape.dimensions[1] / 2., 2.)));

        disc_counter++;

        data_.dynamic_obstacles_.back().pose_ = obstacle.pose;
    }

    LMPCC_INFO_STREAM("Carla Interface: Converted " << data_.dynamic_obstacles_.size() << " obstacles to LMPCC format");

    // PlotReceivedObstacles();
    PlotObstacles(ros_predicted_obstacle_markers_.get(), true, true);

    PostProcessObstacles(&CarlaInterface::ObstacleRejectFunction);

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

// Reject obstacles behind the vehicle
bool CarlaInterface::ObstacleRejectFunction(const Eigen::Vector2d &vehicle_pos, const Eigen::Vector2d &obstacle_pos)
{
    return obstacle_pos(0) < vehicle_pos(0) - 1.0;
}

// External: disabled
void CarlaInterface::halfspaceCallback(const lmpcc_msgs::halfspace_array &msg)
{
    // LMPCC_INFO("Carla Interface::HalfspaceCallback");
    // if (config_->halfspaces_from_spline_)
    // {
    //     LMPCC_INFO("Carla Interface: Using halfspaces from spline, received halfspaces are ignored.");
    //     return;
    // }

    // if ((int)msg.halfspaces.size() > config_->n_halfspaces_)
    //     throw std::runtime_error("Carla Interface: Received static halfspaces exceeded the specified size in n_halfspaces!");

    // LMPCC_INFO_STREAM("Carla Interface: Received " << msg.halfspaces.size() << " halfspaces");

    // // Save halfspaces in the message
    // for (size_t i = 0; i < msg.halfspaces.size(); i++)
    // {
    //     linear_constraints_.halfspaces[i] = msg.halfspaces[i];
    // }

    // // Fill the rest with dummies
    // for (size_t i = msg.halfspaces.size(); (int)i < config_->n_halfspaces_; i++)
    // {
    //     linear_constraints_.halfspaces[i] = dummy_halfspace_;
    // }

    // PlotStraightRoad();
}

void CarlaInterface::PlotStraightRoad()
{

    // // // Plots the road, if it is straight
    // RosTools::ROSLine &road = ros_markers_->getNewLine();
    // road.setScale(0.15, 0.15);
    // road.setColor(0.1, 0.1, 0.1);

    // double length = 100;

    // for(size_t i = 0; i < linear_constraints_.halfspaces.size(); i++){
    //     Eigen::Vector3d point_bot(0.0, linear_constraints_.halfspaces[i].b * linear_constraints_.halfspaces[i].A[1], 0.1);
    //     Eigen::Vector3d point_top(length, linear_constraints_.halfspaces[i].b * linear_constraints_.halfspaces[i].A[1], 0.1);

    //     road.addLine(point_bot, point_top);
    // }

    // ros_markers_->publish();
}

void CarlaInterface::StateCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    LMPCC_INFO("State Callback");
    monitor_->MarkReceived("Pose");

    // Update the frame
    config_->target_frame_ = msg->header.frame_id;

    // Update the states
    solver_interface_ptr_->State().set_psi(RosTools::quaternionToAngle(msg->pose.pose));
    solver_interface_ptr_->State().set_x(msg->pose.pose.position.x + 0 * cos(solver_interface_ptr_->State().psi())); // for shifting the current coordinates to the center of mass
    solver_interface_ptr_->State().set_y(msg->pose.pose.position.y + 0 * sin(solver_interface_ptr_->State().psi()));

    // Plots a Axis to display the path variable location
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = config_->target_frame_;
    transformStamped.child_frame_id = "robot_state";

    transformStamped.transform.translation.x = solver_interface_ptr_->State().x();
    transformStamped.transform.translation.y = solver_interface_ptr_->State().y();
    transformStamped.transform.translation.z = 0.0;
    // tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;

    // Only publish once on the same time stamp (possibly move all this to lmpcc)
    if (last_transform_time_ < transformStamped.header.stamp)
    {
        state_pub_.sendTransform(transformStamped);
        last_transform_time_ = transformStamped.header.stamp;
    }
    // Update the velocity
    solver_interface_ptr_->State().set_v(std::sqrt(std::pow(msg->twist.twist.linear.x, 2) + std::pow(msg->twist.twist.linear.y, 2)));
    controller_->velocity_ = solver_interface_ptr_->State().v();

    t_state_received_ = msg->header.stamp;

    // Update acceleration by numeric differentiation
    double new_velocity_time = msg->header.stamp.toSec(); // ros::Time::now().toSec();

    if (new_velocity_time != last_velocity_time_)
    {

        // Moving average
        // solver_interface_ptr_->State().set_ax((solver_interface_ptr_->State().get_ax() * 2 +
        //                                 ((solver_interface_ptr_->State().get_v() - last_velocity_) / (new_velocity_time - last_velocity_time_)))
        //                                 / 3);
        // solver_interface_ptr_->State().set_a(((solver_interface_ptr_->State().get_v() - last_velocity_) / 0.1));//(new_velocity_time - last_velocity_time_)));
        last_velocity_ = solver_interface_ptr_->State().v();
        last_velocity_time_ = new_velocity_time;
    }

    // // Check whether the waypoints that we are meant to have are ready
    // if (!goal_set_)
    // {
    //     if (data_.path_.Get().x_.size() == 0 ||
    //         Helpers ::dist(Eigen::Vector2d(data_.path_.Get().x_.back(), data_.path_.Get().y_.back()), goal_location_) > 25)
    //     {
    //         PublishGoal();
    //         goal_set_ = true;
    //     }
    // }

    if (config_->debug_output_)
        solver_interface_ptr_->State().print();

    controller_->OnStateReceived();
}

void CarlaInterface::referenceVelocityCallback(const std_msgs::Float64 &msg)
{
    LMPCC_INFO("Carla Interface: Reference Velocity Callback");
    reference_velocity_ = msg.data;
    external_velocity_set_ = true;

    controller_->OnWeightsReceived();
}

void CarlaInterface::contouringWeightCallback(const std_msgs::Float64 &msg)
{
    LMPCC_INFO("Carla Interface: Contouring Weight Callback");
    contouring_weight_ = msg.data;
    external_velocity_set_ = true;

    controller_->OnWeightsReceived();
}

// Callback for ax and ay
void CarlaInterface::AccelerationCallback(const geometry_msgs::AccelWithCovarianceStamped &msg)
{
    LMPCC_INFO("Acceleration Callback");

    // solver_interface_ptr_->State().set_a(msg.accel.accel.linear.x);
    // solver_interface_ptr_->State().set_a(msg.accel.accel.linear.y);
}

void CarlaInterface::AckermannCallback(const carla_ackermann_control::EgoVehicleControlInfo &msg)
{
    data_saver_.AddData("target", msg.target.accel);
    data_saver_.AddData("current", msg.current.accel);

    data_saver_.AddData("target_v", msg.target.speed);
    data_saver_.AddData("current_v", msg.current.speed);

    data_saver_.AddData("pose", Eigen::Vector2d(solver_interface_ptr_->State().x(), solver_interface_ptr_->State().y()));
}

void CarlaInterface::DeployEmergencyStrategy(double deceleration) // Euler roll-out under braking and no-steering assumptions
{
    // Ensure positivity
    deceleration = 1.0 * std::abs(deceleration);
    double vel = solver_interface_ptr_->State().v();
    if (vel <= 0) // Don't start with negative velocity
        vel = 0.;

    // Copy the first state to our backup plan
    solver_interface_ptr_->Plan(0) = solver_interface_ptr_->State(); // Initial state is the current measurement
    // solver_interface_ptr_->Plan(0).x() += std::cos(solver_interface_ptr_->Plan(0).psi()) * vel * solver_interface_ptr_->DT;
    // solver_interface_ptr_->Plan(0).y() += std::sin(solver_interface_ptr_->Plan(0).psi()) * vel * solver_interface_ptr_->DT; // update with velocity

    double x, y, delta, psi, spline;
    x = solver_interface_ptr_->Plan(0).x();
    y = solver_interface_ptr_->Plan(0).y();
    psi = solver_interface_ptr_->Plan(0).psi();
    // delta = solver_interface_ptr_->Plan(0).delta_in();
    delta = solver_interface_ptr_->Plan(0).delta();
    spline = solver_interface_ptr_->spline(0);

    double lr = 1.577;
    double lf = 1.123;
    double ratio = lr / (lr + lf);
    double beta = std::atan(ratio * std::tan(delta)); // Compute the steering under a no-steering assumption

    // Define a backup trajectory as constant deceleration
    for (size_t k = 0; k < solver_interface_ptr_->FORCES_N; k++)
    {
        double prev_vel = vel;
        vel -= deceleration * solver_interface_ptr_->DT; // v -= a*dt (Braking)

        // No backwards driving
        if (vel <= 0.)
        {
            deceleration = prev_vel / solver_interface_ptr_->DT;
            vel = 0.;
        }

        // Update x, y, psi (Model equations)
        x += vel * std::cos(psi + beta) * solver_interface_ptr_->DT;
        y += vel * std::sin(psi + beta) * solver_interface_ptr_->DT;
        psi += vel / lr * std::sin(beta) * solver_interface_ptr_->DT;
        spline += vel * solver_interface_ptr_->DT;

        solver_interface_ptr_->Plan(k).set_x(x);
        solver_interface_ptr_->Plan(k).set_y(y);
        solver_interface_ptr_->Plan(k).set_psi(psi);
        solver_interface_ptr_->x(k) = x;
        solver_interface_ptr_->y(k) = y;
        solver_interface_ptr_->psi(k) = psi;

        solver_interface_ptr_->a(k) = deceleration;
        solver_interface_ptr_->w(k) = 0.;

        solver_interface_ptr_->spline(k) = spline;
    }
}

// void CarlaInterface::WaypointsCallback(const nav_msgs::Path &msg)
// {
//     // if (config_->activate_debug_output_)
//     LMPCC_INFO_STREAM("Carla Interface: Received " << std::floor(0.5 * msg.poses.size()) << " Waypoints");
//     monitor_->MarkReceived("Waypoints");

//     // If new waypoints are close to the old don't do anything
//     // bool waypoints_are_the_same = data_.path_.Get().x_.size() == msg.poses.size();
//     bool waypoints_are_the_same = true;
//     for (size_t i = 0; i < std::floor(0.5 * msg.poses.size()); i++)
//     {
//         if (std::sqrt(std::pow(msg.poses[i * 2].pose.position.x - data_.path_.Get().x_[i], 2) +
//                       std::pow(msg.poses[i * 2].pose.position.y - data_.path_.Get().y_[i], 2)) > 1.0)
//         {
//             waypoints_are_the_same = false;
//             break;
//         }
//     }

//     // Do not update the reference path if the received path is the same (to reduce computation times)
//     if (waypoints_are_the_same)
//         return;

//     data_.path_.Get().Clear();

//     for (size_t i = 0; i < msg.poses.size(); i += 2)
//         data_.path_.Get().AddPose(msg.poses[i].pose);

//     controller_->OnWaypointsReceived();
// }

// Callback for detecting slow down of the Carla ROS bridge
void CarlaInterface::carlaStatusCallback(const carla_msgs::CarlaStatus &msg)
{

    static RosTools::Benchmarker carla_benchmarker("Carla Frames"); // Print min/max/avg timing of frames of Carla

    // Not nice, but just to ignore the first few frames, because they are slow
    static bool hasReset;
    if (!hasReset && carla_benchmarker.getTotalRuns() > 200)
    {
        carla_benchmarker.stop();
        carla_benchmarker.reset();
        hasReset = true;
    }

    if (carla_benchmarker.isRunning())
    {
        double last_duration = carla_benchmarker.stop();

        LMPCC_INFO_STREAM("Carla Frame Update Took: " << last_duration * 1000 << " ms, configured delta: " << msg.fixed_delta_seconds * 1000 << " ms");
    }

    carla_benchmarker.start();
}

// Reset when a collision is detected
void CarlaInterface::carlaCollisionCallback(const carla_msgs::CarlaCollisionEvent &msg)
{
    if (collision_detected_)
        return;

    LMPCC_WARN_ALWAYS("Collision detected. Resetting...");
    collision_detected_ = true;

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.pose.pose = reset_pose_;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = config_->target_frame_;
    ResetCallback(pose);
}

void CarlaInterface::PlotPredictedObstacles()
{
    LMPCC_INFO("Carla Interface: Plot Predicted Obstacles");

    RosTools::ROSPointMarker &obstacle_circles = ros_predicted_obstacle_markers_->getNewPointMarker("CYLINDER");
    obstacle_circles.setZ(-0.1);

    // Visualize selected obstacles
    for (auto &obstacle : data_.dynamic_obstacles_)
    {
        for (auto &disc : obstacle.discs_)
        {
            obstacle_circles.setColor(0.0, 1.0, 0.0, 0.3);

            // Plot each disc of the obstacle
            Eigen::Vector2d disc_pose = disc.AsVector2d();

            // Set the received radius
            obstacle_circles.setScale(
                2 * disc.radius,
                2 * disc.radius,
                0.1);
            obstacle_circles.addPointMarker(Eigen::Vector3d(disc_pose(0), disc_pose(1), 0.05));
        }
    }

    ros_predicted_obstacle_markers_->publish();
}
void CarlaInterface::PlotReceivedObstacles()
{

    LMPCC_WARN("Carla Interface: Plotting Received Obstacles");
    double plot_height = 0.05;
    RosTools::ROSPointMarker &obstacle_marker = ros_all_obstacle_markers_->getNewPointMarker("CYLINDER");
    obstacle_marker.setScale(0.25, 0.25, plot_height);

    // Debug plot option to see the bounding boxes
    bool plot_boxes = false;
    RosTools::ROSPointMarker &obstacle_boxes = ros_all_obstacle_markers_->getNewPointMarker("CUBE");
    obstacle_boxes.setColor(1.0, 0.0, 0.0, 0.8);
    obstacle_boxes.setZ(1.0);

    // Plot all obstacles
    // for (auto &object : sorted_obstacles_.objects)
    // {
    for (DynamicObstacle &obstacle : data_.dynamic_obstacles_)
    {
        // Plot boxes shows the dimensions that carla sends us
        if (plot_boxes)
        {
            obstacle_boxes.setColor(1.0, 0.0, 0.0, 0.8);

            obstacle_boxes.setScale(0.1, 0.1, 2.0);
            obstacle_boxes.addPointMarker(obstacle.pose_);
        }

        // This marker is a simple cube at the exact position.
        // obstacle_marker.setColorInt(obstacle.id_ % VIRIDIS_COLORS + 12, 0.7); // Each obstacle a different color (NOT VISIBLE ENOUGH)
        // obstacle_marker.setColor(247. / 256., 177. / 256., 64. / 256., 0.8);
        obstacle_marker.setColor(0.0, 0.0, 0.0, 0.6);

        // Colors on cylinders do not seem to be respected in the camera...

        obstacle_marker.setScale(obstacle.discs_[0].radius * 2, obstacle.discs_[0].radius * 2, plot_height);
        obstacle.pose_.position.z = plot_height / 2.;
        obstacle_marker.addPointMarker(obstacle.pose_);
    }

    ros_all_obstacle_markers_->publish();
}

void CarlaInterface::carlaGoalPositionCallback(const geometry_msgs::PoseStamped &msg)
{
    // time_ = msg.header.stamp.sec - time_;
    // time_taken_to_finish_.AddData("TimeTaken", time_);
    // if (time_ > 20)
    // {
    //     time_taken_to_finish_.SaveData("Time");
    // }
}
