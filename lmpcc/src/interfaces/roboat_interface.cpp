#include "interfaces/roboat_interface.h"
#include "lmpcc_controller.h"

RoboatInterface::RoboatInterface(ros::NodeHandle &nh, MPCC *controller, predictive_configuration *config, SolverInterface *solver_interface_ptr)
    : Interface(nh, controller, config, solver_interface_ptr)
{
    ROS_INFO("Initializing Roboat Interface...");

    // Subscribers for sensor data
    state_sub_ = nh.subscribe(config_->robot_state_topic_, 1, &RoboatInterface::StateCallBack, this);

    // Subscribers for high level data
    obstacle_sub_ = nh.subscribe(config_->obs_state_topic_, 1, &RoboatInterface::ObstacleCallBack, this);
    linearconstraints_sub_ = nh.subscribe(config_->linear_constraints_topic_, 1, &RoboatInterface::LinearConstraintsCallback, this);

    // Publisher for vehicle command
    command_pub_ = nh.advertise<lmpcc_msgs::Force>(config_->cmd_, 1);

    // Publisher for resetting
    reset_roboat_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(config_->reset_environment_topic_, 1);
    reset_sub_ = nh.subscribe(config_->reset_topic_, 1, &RoboatInterface::ResetCallback, this);

    // Publisher for visualizing obstacles
    obstacles_pub_ = nh.advertise<visualization_msgs::MarkerArray>(config_->obstacles_received_topic_, 1);
    repulsive_ellipsoids_pub = nh.advertise<visualization_msgs::MarkerArray>(config_->repulsive_ellipsoids_topic_, 1);
    repulsive_regulations_pub = nh.advertise<visualization_msgs::MarkerArray>(config_->repulsive_regulations_topic_, 1);
    repulsive_rightofway_pub = nh.advertise<visualization_msgs::MarkerArray>(config_->repulsive_rightofway_topic_, 1);

    // Subscribe to waypoints
    waypoints_sub_ = nh.subscribe(config_->waypoint_topic_, 1, &RoboatInterface::WaypointsCallback, this);

    // Service clients for resetting
    reset_simulation_client_ = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
    reset_ekf_client_ = nh.serviceClient<robot_localization::SetPose>("/set_pose");

    goal_set_ = false; // Publish when the waypoint publisher is online
    goal_location_ = Eigen::Vector2d(324, -2);

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
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[t].pose.position.x = 50;
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[t].pose.position.y = 50;
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[t].pose.orientation.z = 0;
            obstacles_.lmpcc_obstacles[obst_it].major_semiaxis[t] = 0.1;
            obstacles_.lmpcc_obstacles[obst_it].minor_semiaxis[t] = 0.1;
        }
    }

    ROS_INFO("Roboat Interface Initialized");
}

void RoboatInterface::Actuate()
{
    lmpcc_msgs::Force force_msg;
    force_msg.priority = 1; // IThrustersManager.h --> TH_CMD_PRIORITY::TRACKING;
    force_msg.allocated = true;

    // Fill msg with forces
    std::vector<double> forces = {0.0, 0.0, 0.0, 0.0};
    forces[0] = solver_interface_ptr_->f1();
    forces[1] = solver_interface_ptr_->f2();
    forces[2] = solver_interface_ptr_->f3();
    forces[3] = solver_interface_ptr_->f4();
    std::copy(forces.begin(), forces.end(), &force_msg.data[0]);

    // Publish the command
    command_pub_.publish(force_msg);
}

void RoboatInterface::ActuateBrake(double deceleration)
{
    lmpcc_msgs::Force force_msg;
    force_msg.priority = 1; // IThrustersManager.h --> TH_CMD_PRIORITY::TRACKING;
    force_msg.allocated = true;

    // Fill msg with forces
    std::vector<double> forces = {0.0, 0.0, 0.0, 0.0};
    std::copy(forces.begin(), forces.end(), &force_msg.data[0]);

    // Publish the command
    command_pub_.publish(force_msg);
}

void RoboatInterface::Reset(const geometry_msgs::Pose &position_after_reset)
{
    std::cout << "roboat interface reset(msg) called." << std::endl;
    geometry_msgs::PoseWithCovarianceStamped reset_msg;
    reset_msg.pose.pose = position_after_reset;

    for (int j = 0; j < 10; j++)
    {
        ActuateBrake(0.0);
        ros::Duration(1.0 / config_->clock_frequency_).sleep();
    }
    reset_roboat_pub_.publish(reset_msg);
    ros::Duration(0.5).sleep();
}

void RoboatInterface::Reset()
{
    std::cout << "roboat interface reset() called." << std::endl;
    reset_simulation_client_.call(reset_msg_);
    reset_ekf_client_.call(reset_pose_msg_);

    geometry_msgs::Pose default_position;
    default_position.position.x = 0;
    default_position.position.y = 0;
    default_position.orientation.z = 0;

    Reset(default_position);
}

/* CALLBACKS */
void RoboatInterface::ResetCallback(const std_msgs::Empty &msg)
{
    if (external_reset_callback_)
        (controller_->*external_reset_callback_)();
}

void RoboatInterface::ObstacleCallBack(const lmpcc_msgs::lmpcc_obstacle_array &received_obstacles)
{
    if (config_->debug_output_)
        ROS_INFO_STREAM("obstacles received (n = " << received_obstacles.lmpcc_obstacles.size() << ")");

    // If there were other objects
    if (received_obstacles.lmpcc_obstacles.size() > 0)
    {

        // If there are less or equal amount of objects as the maximum configured number of obstacles
        // Take the first received obstacles
        // And fill up with dummy obstacles
        if (received_obstacles.lmpcc_obstacles.size() < (unsigned int)config_->max_obstacles_)
        {

            if (config_->debug_output_)
                ROS_INFO_STREAM("Roboat Interface: Received " << received_obstacles.lmpcc_obstacles.size() - 1
                                                              << " Obstacles (Less than max_obstacles="
                                                              << config_->max_obstacles_ << ")");
            for (int j = 0; (unsigned int)j < received_obstacles.lmpcc_obstacles.size(); j++)
            {
                obstacles_.lmpcc_obstacles[j] = received_obstacles.lmpcc_obstacles[j];
            }

            for (int j = received_obstacles.lmpcc_obstacles.size(); j < config_->max_obstacles_; j++)
            {
                obstacles_.lmpcc_obstacles[j].id = 10 + j;
                obstacles_.lmpcc_obstacles[j].pose.position.x = solver_interface_ptr_->State().x() + 500;
                obstacles_.lmpcc_obstacles[j].pose.position.y = solver_interface_ptr_->State().y() + 500;

                for (size_t t = 0; t < solver_interface_ptr_->FORCES_N; t++)
                {
                    obstacles_.lmpcc_obstacles[j].trajectory.poses[t].pose.position.x = solver_interface_ptr_->State().x() + 500;
                    obstacles_.lmpcc_obstacles[j].trajectory.poses[t].pose.position.y = solver_interface_ptr_->State().y() + 500;
                    obstacles_.lmpcc_obstacles[j].trajectory.poses[t].pose.orientation.z = 0;
                    obstacles_.lmpcc_obstacles[j].major_semiaxis[t] = 0.01;
                    obstacles_.lmpcc_obstacles[j].minor_semiaxis[t] = 0.01;
                }
            }
        }

        // If the amount of received obstacles is exactly the same, we can just use the received message
        else if (received_obstacles.lmpcc_obstacles.size() == (unsigned int)config_->max_obstacles_)
        {
            obstacles_ = received_obstacles;
        }

        // If there are more obstacles, order them
        else
        {
            obstacles_ = received_obstacles;
            OrderObstacles(obstacles_);
        }
    }

    // If there are no obstacles, insert dummies
    else
    {
        if (config_->debug_output_)
            ROS_INFO_STREAM("Roboat Interface: Received no obstacles, inserting dummies");

        for (int j = 0; j < config_->max_obstacles_; j++)
        {
            obstacles_.lmpcc_obstacles[j].id = 10 + j;
            obstacles_.lmpcc_obstacles[j].pose.position.x = solver_interface_ptr_->State().x() + 500;
            obstacles_.lmpcc_obstacles[j].pose.position.y = solver_interface_ptr_->State().y() + 500;

            for (size_t t = 0; t < solver_interface_ptr_->FORCES_N; t++)
            {
                obstacles_.lmpcc_obstacles[j].trajectory.poses[t].pose.position.x = solver_interface_ptr_->State().x() + 500;
                obstacles_.lmpcc_obstacles[j].trajectory.poses[t].pose.position.y = solver_interface_ptr_->State().y() + 500;
                obstacles_.lmpcc_obstacles[j].trajectory.poses[t].pose.orientation.z = 0;
                obstacles_.lmpcc_obstacles[j].major_semiaxis[t] = 0.01;
                obstacles_.lmpcc_obstacles[j].minor_semiaxis[t] = 0.01;
            }
        }
    }

    if (config_->debug_output_)
    {
        for (size_t i = 0; i < obstacles_.lmpcc_obstacles.size(); i++)
            ROS_WARN_STREAM("Roboat Interface: Distance to obstacle: " << obstacles_.lmpcc_obstacles[i].distance);
    }

    determinePriorityVessels(obstacles_);
    plotObstacles();

    if (solver_interface_ptr_->enable_ellipsoid_constraints)
    {
        visualizeRepulsiveEllipsoidCosts();
    }
    if (solver_interface_ptr_->enable_waterregulations_costs)
    {
        visualizeRepulsiveRegulationCosts();
    }

    if (solver_interface_ptr_->enable_rightofway_costs)
    {
        visualizeRepulsiveRightofWay();
    }

    if (external_obstacle_callback_)
        (controller_->*external_obstacle_callback_)();
}

void RoboatInterface::OrderObstacles(lmpcc_msgs::lmpcc_obstacle_array &ellipses)
{
    // Create vector of obstacles
    if (ellipses.lmpcc_obstacles.size() > 0)
    {
        std::vector<lmpcc_msgs::lmpcc_obstacle> ellipsesVector;
        ellipsesVector = ellipses.lmpcc_obstacles;

        // Sort vector according to distances
        std::sort(ellipsesVector.begin(), ellipsesVector.end(), [](lmpcc_msgs::lmpcc_obstacle const &obst1, lmpcc_msgs::lmpcc_obstacle const &obst2)
                  { return (obst1.distance < obst2.distance); });

        // Write vector of sorted obstacles to obstacles structure
        ellipses.lmpcc_obstacles = ellipsesVector;
    }
}

void RoboatInterface::LinearConstraintsCallback(lmpcc_msgs::halfspace_array linearconstraints_msg)
{
    // Oscar: for all k
    for (u_int k = 0; k < solver_interface_ptr_->FORCES_N; k++)
        linear_constraints_[k] = linearconstraints_msg;
}

void RoboatInterface::StateCallBack(const nav_msgs::Odometry &msg)
{
    if (config_->debug_output_)
        ROS_INFO("Roboat Interface: State Callback");

    // Update the frame
    config_->target_frame_ = msg.header.frame_id;

    // Update the states
    solver_interface_ptr_->State().set_x(msg.pose.pose.position.x);
    solver_interface_ptr_->State().set_y(msg.pose.pose.position.y);
    solver_interface_ptr_->State().set_psi(RosTools::quaternionToAngle(msg.pose.pose.orientation));
    solver_interface_ptr_->State().set_u(msg.twist.twist.linear.x);
    solver_interface_ptr_->State().set_v(msg.twist.twist.linear.y);
    solver_interface_ptr_->State().set_r(msg.twist.twist.angular.z);

    // Plots a Axis to display the path variable location
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = config_->target_frame_;
    transformStamped.child_frame_id = "robot_state";

    transformStamped.transform.translation.x = solver_interface_ptr_->State().x();
    transformStamped.transform.translation.y = solver_interface_ptr_->State().y();
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;

    path_pose_pub_.sendTransform(transformStamped);

    // Check whether the waypoints that we are meant to have are ready
    if (!goal_set_)
    {
        if (x_.size() == 0 || Helpers ::dist(Eigen::Vector2d(x_[x_.size() - 1], y_[y_.size() - 1]), goal_location_) > 25)
        {
            goal_set_ = true;
        }
    }

    if (config_->debug_output_)
        solver_interface_ptr_->State().print();

    if (external_state_callback_)
        (controller_->*external_state_callback_)();
}

void RoboatInterface::WaypointsCallback(const nav_msgs::Path &msg)
{
    if (config_->debug_output_)
        ROS_INFO_STREAM("Roboat Interface: Waypoint callback with " << msg.poses.size() << " waypoints");

    int waypoint_count = 0;

    psi_.clear();
    x_.clear();
    y_.clear();

    for (size_t i = 0; i < msg.poses.size(); i += 2)
    {
        psi_.push_back(RosTools::quaternionToAngle(msg.poses[i].pose));
        x_.push_back(msg.poses[i].pose.position.x);
        y_.push_back(msg.poses[i].pose.position.y);

        waypoint_count++;
    }

    if (external_waypoints_callback_)
        (controller_->*external_waypoints_callback_)();
}

void RoboatInterface::plotObstacles(void)
{
    visualization_msgs::MarkerArray obstacles_list;

    unsigned int n = std::min((unsigned int)config_->max_obstacles_, (unsigned int)obstacles_.lmpcc_obstacles.size());

    for (size_t obs_id = 0; obs_id < n; obs_id++) // 100 points
    {
        visualization_msgs::Marker obs_shape;

        obs_shape.header.frame_id = config_->target_frame_;
        obs_shape.id = 100 + obs_id;

        if (obstacles_.lmpcc_obstacles[obs_id].id == 6)
        {
            obs_shape.type = visualization_msgs::Marker::CUBE;
        }
        else
        {
            obs_shape.type = visualization_msgs::Marker::SPHERE;
        }
        obs_shape.type = visualization_msgs::Marker::SPHERE;
        obs_shape.scale.x = obstacles_.lmpcc_obstacles[obs_id].major_semiaxis[0] * 1.25;
        obs_shape.scale.y = obstacles_.lmpcc_obstacles[obs_id].minor_semiaxis[0] * 1.25;
        obs_shape.scale.z = 0.2;
        // Line strip is blue
        obs_shape.color.r = 0.2;
        obs_shape.color.g = 0.2;
        obs_shape.color.b = 0.7;
        obs_shape.color.a = 0.8;

        obs_shape.pose.position = obstacles_.lmpcc_obstacles[obs_id].pose.position;
        obs_shape.pose.orientation = tf::createQuaternionMsgFromYaw(obstacles_.lmpcc_obstacles[obs_id].pose.orientation.z);

        obstacles_list.markers.push_back(obs_shape);
    }

    for (size_t obs_id = 0; obs_id < n; obs_id++) // 100 points
    {
        for (size_t t = 0; t < solver_interface_ptr_->FORCES_N; t = t + 3) // 100 points
        {
            visualization_msgs::Marker obs_shape;

            obs_shape.header.frame_id = config_->target_frame_;
            obs_shape.id = 200 + obs_id * solver_interface_ptr_->FORCES_N + t;

            if (obstacles_.lmpcc_obstacles[obs_id].id == 6)
            {
                obs_shape.type = visualization_msgs::Marker::CUBE;
            }
            else
            {
                obs_shape.type = visualization_msgs::Marker::SPHERE;
            }
            obs_shape.type = visualization_msgs::Marker::SPHERE;
            obs_shape.scale.x = obstacles_.lmpcc_obstacles[obs_id].major_semiaxis[0] * 1.25;
            obs_shape.scale.y = obstacles_.lmpcc_obstacles[obs_id].minor_semiaxis[0] * 1.25;
            obs_shape.scale.z = 0.2;

            obs_shape.color.r = 0.8;
            obs_shape.color.g = 0.2;
            obs_shape.color.b = 0.2;
            obs_shape.color.a = 1.0 / (1 + t);

            obs_shape.pose = obstacles_.lmpcc_obstacles[obs_id].trajectory.poses[t].pose;

            obs_shape.pose.position = obstacles_.lmpcc_obstacles[obs_id].trajectory.poses[t].pose.position;
            obs_shape.pose.orientation = tf::createQuaternionMsgFromYaw(obstacles_.lmpcc_obstacles[obs_id].trajectory.poses[t].pose.orientation.z);
            // ROS_INFO_STREAM("obs_id: " << obs_id << " t: " << t);
            obstacles_list.markers.push_back(obs_shape);
        }
    }

    obstacles_pub_.publish(obstacles_list);
}

template <typename T>
inline constexpr int signum(T x, std::false_type is_signed)
{
    return T(0) < x;
}

template <typename T>
inline constexpr int signum(T x, std::true_type is_signed)
{
    return (T(0) < x) - (x < T(0));
}

template <typename T>
inline constexpr int signum(T x)
{
    return signum(x, std::is_signed<T>());
}

void RoboatInterface::determinePriorityVessels(const lmpcc_msgs::lmpcc_obstacle_array &obstacles)
{

    // Roboat information
    double roboat_x = solver_interface_ptr_->State().x();
    double roboat_y = solver_interface_ptr_->State().y();
    double roboat_psi = solver_interface_ptr_->State().psi();

    priority_vessels_.clear();

    for (int i = 0; (unsigned int)i < obstacles.lmpcc_obstacles.size(); i++)
    {

        // Obstacle information
        int id_boat = obstacles.lmpcc_obstacles[i].id;
        double major_axis = obstacles.lmpcc_obstacles[i].major_semiaxis[0];
        double minor_axis = obstacles.lmpcc_obstacles[i].minor_semiaxis[0];

        bool priority;
        // If the boat is longer than 20 meters we need to give way
        // TODO(@Jitske): or if it is a priority vessel (for example canal cruise boats)
        if (major_axis > 20 || minor_axis > 20)
        {
            priority = true;
        }
        else
        {
            // More obstacle information
            double obst_x = obstacles.lmpcc_obstacles[i].pose.position.x;
            double obst_y = obstacles.lmpcc_obstacles[i].pose.position.y;
            double obst_psi = obstacles.lmpcc_obstacles[i].pose.orientation.z;

            // Calculate relative positions
            // In World Frame
            double x_roboat_to_obst_W = obst_x - roboat_x;
            double y_roboat_to_obst_W = obst_y - roboat_y;
            // In Roboat Body Frame
            double x_obst_RB = cos(-roboat_psi) * x_roboat_to_obst_W - sin(-roboat_psi) * y_roboat_to_obst_W;
            double y_obst_RB = sin(-roboat_psi) * x_roboat_to_obst_W + cos(-roboat_psi) * y_roboat_to_obst_W;
            // In Obstacle Body Frame
            double x_roboat_OB = cos(-obst_psi) * (-x_roboat_to_obst_W) - sin(-obst_psi) * (-y_roboat_to_obst_W);
            double y_roboat_OB = sin(-obst_psi) * (-x_roboat_to_obst_W) + cos(-obst_psi) * (-y_roboat_to_obst_W);

            // Calculate angles alpha and beta
            double alpha = signum(y_obst_RB) * acos(x_obst_RB / sqrt(pow(x_obst_RB, 2) + pow(y_obst_RB, 2)));
            double beta = signum(y_roboat_OB) * acos(x_roboat_OB / sqrt(pow(x_roboat_OB, 2) + pow(y_roboat_OB, 2)));

            // Maximum and minimum values for alpha and beta
            double pi = M_PI;
            double lower_angle = pi / 30;
            double upper_angle = pi * 5 / 8;

            // Calculate if priority
            // We need to give priority if the boat is comming from the right and is moving towards our path
            if (-upper_angle <= alpha && alpha <= -lower_angle && lower_angle <= beta && beta <= upper_angle)
            {
                priority = true;
            }
            else
            {
                priority = false;
            }
        }
        priority_vessels_.insert(std::pair<int, bool>(id_boat, priority));
    }
}

void RoboatInterface::visualizeRepulsiveEllipsoidCosts()
{
    visualization_msgs::MarkerArray obstacles_list;

    unsigned int n = std::min((unsigned int)config_->max_obstacles_, (unsigned int)obstacles_.lmpcc_obstacles.size());
    double r_discs_ = controller_->r_discs_;

    double maximum_weight = 20;
    double weight_brightness = controller_->repulsive_weight_ / maximum_weight;

    for (size_t obs_id = 0; obs_id < n; obs_id++) // 100 points
    {
        visualization_msgs::Marker obs_shape;

        obs_shape.header.frame_id = config_->target_frame_;
        obs_shape.id = 100 + obs_id;

        if (obstacles_.lmpcc_obstacles[obs_id].major_semiaxis[0] > 0.02 && obstacles_.lmpcc_obstacles[obs_id].minor_semiaxis[0] > 0.02)
        {
            obs_shape.type = visualization_msgs::Marker::SPHERE;
            obs_shape.scale.x =
                obstacles_.lmpcc_obstacles[obs_id].major_semiaxis[0] + controller_->margin_ellipsoid_ + r_discs_;
            obs_shape.scale.y =
                obstacles_.lmpcc_obstacles[obs_id].minor_semiaxis[0] + controller_->margin_ellipsoid_ + r_discs_;
            obs_shape.scale.z = 0.2;
            // Line strip is orange
            obs_shape.color.r = 1.0;
            obs_shape.color.g = 0.6;
            obs_shape.color.b = 0.0;
            obs_shape.color.a = weight_brightness;

            obs_shape.pose.position = obstacles_.lmpcc_obstacles[obs_id].pose.position;
            obs_shape.pose.orientation = tf::createQuaternionMsgFromYaw(
                obstacles_.lmpcc_obstacles[obs_id].pose.orientation.z);
        }
        else
        {
            obs_shape.action = visualization_msgs::Marker::DELETE;
        }

        obstacles_list.markers.push_back(obs_shape);
    }

    for (size_t obs_id = 0; obs_id < n; obs_id++) // 100 points
    {
        for (size_t t = 0; t < solver_interface_ptr_->FORCES_N; t = t + 3) // 100 points
        {
            visualization_msgs::Marker obs_shape;

            obs_shape.header.frame_id = config_->target_frame_;
            obs_shape.id = 200 + obs_id * solver_interface_ptr_->FORCES_N + t;

            if (obstacles_.lmpcc_obstacles[obs_id].major_semiaxis[0] > 0.02 && obstacles_.lmpcc_obstacles[obs_id].minor_semiaxis[0] > 0.02)
            {
                obs_shape.type = visualization_msgs::Marker::SPHERE;
                obs_shape.scale.x =
                    obstacles_.lmpcc_obstacles[obs_id].major_semiaxis[0] + controller_->margin_ellipsoid_ +
                    r_discs_;
                obs_shape.scale.y =
                    obstacles_.lmpcc_obstacles[obs_id].minor_semiaxis[0] + controller_->margin_ellipsoid_ +
                    r_discs_;
                obs_shape.scale.z = 0.2;

                obs_shape.color.r = 1.0;
                obs_shape.color.g = 0.6;
                obs_shape.color.b = 0.0;
                obs_shape.color.a = weight_brightness / (1.0 + t);

                obs_shape.pose = obstacles_.lmpcc_obstacles[obs_id].trajectory.poses[t].pose;

                obs_shape.pose.position = obstacles_.lmpcc_obstacles[obs_id].trajectory.poses[t].pose.position;
                obs_shape.pose.orientation = tf::createQuaternionMsgFromYaw(
                    obstacles_.lmpcc_obstacles[obs_id].trajectory.poses[t].pose.orientation.z);
            }
            else
            {
                obs_shape.action = visualization_msgs::Marker::DELETE;
            }
            obstacles_list.markers.push_back(obs_shape);
        }
    }

    repulsive_ellipsoids_pub.publish(obstacles_list);
}

void RoboatInterface::visualizeRepulsiveRegulationCosts()
{
    visualization_msgs::MarkerArray obstacles_list;

    double a_ellipse_factor = controller_->a_ellipse_factor_;
    double b_ellipse_factor = controller_->b_ellipse_factor_;
    double c_ellipse_factor = controller_->c_ellipse_factor_;
    double d_ellipse_factor = controller_->d_ellipse_factor_;
    double r_discs = controller_->r_discs_;

    unsigned int n = std::min((unsigned int)config_->max_obstacles_, (unsigned int)obstacles_.lmpcc_obstacles.size());

    double maximum_weight = 20;
    double weight_brightness = controller_->W_ellipseregulation_ / maximum_weight;

    for (size_t obs_id = 0; obs_id < n; obs_id++) // 100 points
    {
        visualization_msgs::Marker obs_shape;

        obs_shape.header.frame_id = config_->target_frame_;
        obs_shape.id = 100 + obs_id;

        if (obstacles_.lmpcc_obstacles[obs_id].major_semiaxis[0] > 0.02 && obstacles_.lmpcc_obstacles[obs_id].minor_semiaxis[0] > 0.02)
        {
            obs_shape.type = visualization_msgs::Marker::SPHERE;
            obs_shape.scale.x = (obstacles_.lmpcc_obstacles[obs_id].major_semiaxis[0] + r_discs) * a_ellipse_factor;
            obs_shape.scale.y = (obstacles_.lmpcc_obstacles[obs_id].minor_semiaxis[0] + r_discs) * b_ellipse_factor;
            obs_shape.scale.z = 0.2;
            // Line strip is orange
            obs_shape.color.r = 1.0;
            obs_shape.color.g = 0.6;
            obs_shape.color.b = 0.0;
            obs_shape.color.a = weight_brightness;

            obs_shape.pose.position = obstacles_.lmpcc_obstacles[obs_id].pose.position;
            obs_shape.pose.orientation = tf::createQuaternionMsgFromYaw(
                obstacles_.lmpcc_obstacles[obs_id].pose.orientation.z);
            double angle = obstacles_.lmpcc_obstacles[obs_id].pose.orientation.z;
            obs_shape.pose.position.x =
                obs_shape.pose.position.x + c_ellipse_factor * cos(angle) + d_ellipse_factor * sin(angle);
            obs_shape.pose.position.y =
                obs_shape.pose.position.y + c_ellipse_factor * sin(angle) - d_ellipse_factor * cos(angle);
        }
        else
        {
            obs_shape.action = visualization_msgs::Marker::DELETE;
        }

        obstacles_list.markers.push_back(obs_shape);
    }

    for (size_t obs_id = 0; obs_id < n; obs_id++) // 100 points
    {
        for (size_t t = 0; t < solver_interface_ptr_->FORCES_N; t = t + 3) // 100 points
        {
            visualization_msgs::Marker obs_shape;

            obs_shape.header.frame_id = config_->target_frame_;
            obs_shape.id = 200 + obs_id * solver_interface_ptr_->FORCES_N + t;

            if (obstacles_.lmpcc_obstacles[obs_id].major_semiaxis[0] > 0.02 && obstacles_.lmpcc_obstacles[obs_id].minor_semiaxis[0] > 0.02)
            {
                obs_shape.type = visualization_msgs::Marker::SPHERE;
                obs_shape.scale.x = (obstacles_.lmpcc_obstacles[obs_id].major_semiaxis[0] + r_discs) * a_ellipse_factor;
                obs_shape.scale.y = (obstacles_.lmpcc_obstacles[obs_id].minor_semiaxis[0] + r_discs) * b_ellipse_factor;
                obs_shape.scale.z = 0.2;

                obs_shape.color.r = 1.0;
                obs_shape.color.g = 0.6;
                obs_shape.color.b = 0.0;
                obs_shape.color.a = weight_brightness / (1.0 + t);

                obs_shape.pose = obstacles_.lmpcc_obstacles[obs_id].trajectory.poses[t].pose;

                obs_shape.pose.position = obstacles_.lmpcc_obstacles[obs_id].trajectory.poses[t].pose.position;
                obs_shape.pose.orientation = tf::createQuaternionMsgFromYaw(
                    obstacles_.lmpcc_obstacles[obs_id].trajectory.poses[t].pose.orientation.z);
                double angle = obstacles_.lmpcc_obstacles[obs_id].trajectory.poses[t].pose.orientation.z;
                obs_shape.pose.position.x =
                    obs_shape.pose.position.x + c_ellipse_factor * cos(angle) + d_ellipse_factor * sin(angle);
                obs_shape.pose.position.y =
                    obs_shape.pose.position.y + c_ellipse_factor * sin(angle) - d_ellipse_factor * cos(angle);
            }
            else
            {
                obs_shape.action = visualization_msgs::Marker::DELETE;
            }
            obstacles_list.markers.push_back(obs_shape);
        }
    }

    repulsive_regulations_pub.publish(obstacles_list);
}

void RoboatInterface::visualizeRepulsiveRightofWay()
{
    visualization_msgs::MarkerArray obstacles_list;

    double e_ellipse_factor = controller_->e_ellipse_factor_;
    double f_ellipse_factor = controller_->f_ellipse_factor_;
    double r_discs = controller_->r_discs_;

    unsigned int n = std::min((unsigned int)config_->max_obstacles_, (unsigned int)obstacles_.lmpcc_obstacles.size());

    double maximum_weight = 20;
    double weight_brightness = controller_->W_rightofway_ / maximum_weight;

    for (size_t obs_id = 0; obs_id < n; obs_id++) // 100 points
    {
        visualization_msgs::Marker obs_shape;

        bool priority_vessel = priority_vessels_[obstacles_.lmpcc_obstacles[obs_id].id];
        obs_shape.header.frame_id = config_->target_frame_;
        obs_shape.id = 100 + obs_id;

        if (priority_vessel)
        {
            obs_shape.type = visualization_msgs::Marker::SPHERE;
            obs_shape.scale.x = e_ellipse_factor;
            obs_shape.scale.y = (obstacles_.lmpcc_obstacles[obs_id].minor_semiaxis[0] + r_discs);
            obs_shape.scale.z = 0.2;
            // Line strip is orange
            obs_shape.color.r = 1.0;
            obs_shape.color.g = 0.6;
            obs_shape.color.b = 0.0;
            obs_shape.color.a = weight_brightness;

            obs_shape.pose.position = obstacles_.lmpcc_obstacles[obs_id].pose.position;
            obs_shape.pose.orientation = tf::createQuaternionMsgFromYaw(
                obstacles_.lmpcc_obstacles[obs_id].pose.orientation.z);
            double angle = obstacles_.lmpcc_obstacles[obs_id].pose.orientation.z;
            obs_shape.pose.position.x = obs_shape.pose.position.x + f_ellipse_factor * cos(angle);
            obs_shape.pose.position.y = obs_shape.pose.position.y + f_ellipse_factor * sin(angle);
        }
        else
        {
            obs_shape.action = visualization_msgs::Marker::DELETE;
        }
        obstacles_list.markers.push_back(obs_shape);
    }

    for (size_t obs_id = 0; obs_id < n; obs_id++) // 100 points
    {
        bool priority_vessel = priority_vessels_[obstacles_.lmpcc_obstacles[obs_id].id];

        for (size_t t = 0; t < solver_interface_ptr_->FORCES_N; t = t + 3) // 100 points
        {
            visualization_msgs::Marker obs_shape;

            obs_shape.header.frame_id = config_->target_frame_;
            obs_shape.id = 200 + obs_id * solver_interface_ptr_->FORCES_N + t;

            if (priority_vessel)
            {

                obs_shape.type = visualization_msgs::Marker::SPHERE;
                obs_shape.scale.x = e_ellipse_factor;
                obs_shape.scale.y = (obstacles_.lmpcc_obstacles[obs_id].minor_semiaxis[0] + r_discs);
                obs_shape.scale.z = 0.2;

                obs_shape.color.r = 1.0;
                obs_shape.color.g = 0.6;
                obs_shape.color.b = 0.0;
                obs_shape.color.a = weight_brightness / (1.0 + t);

                obs_shape.pose = obstacles_.lmpcc_obstacles[obs_id].trajectory.poses[t].pose;

                obs_shape.pose.position = obstacles_.lmpcc_obstacles[obs_id].trajectory.poses[t].pose.position;
                obs_shape.pose.orientation = tf::createQuaternionMsgFromYaw(
                    obstacles_.lmpcc_obstacles[obs_id].trajectory.poses[t].pose.orientation.z);
                double angle = obstacles_.lmpcc_obstacles[obs_id].trajectory.poses[t].pose.orientation.z;
                obs_shape.pose.position.x = obs_shape.pose.position.x + f_ellipse_factor * cos(angle);
                obs_shape.pose.position.y = obs_shape.pose.position.y + f_ellipse_factor * sin(angle);
            }
            else
            {
                obs_shape.action = visualization_msgs::Marker::DELETE;
            }
            obstacles_list.markers.push_back(obs_shape);
        }
    }

    repulsive_rightofway_pub.publish(obstacles_list);
}