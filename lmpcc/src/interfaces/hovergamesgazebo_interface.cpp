#include "interfaces/hovergamesgazebo_interface.h"
#include "lmpcc_controller.h"

HovergamesGazeboInterface::HovergamesGazeboInterface(ros::NodeHandle &nh, MPCC *controller, predictive_configuration *config, SolverInterface *solver_interface_ptr)
    : Interface(nh, controller, config, solver_interface_ptr)
{
    LMPCC_WARN_ALWAYS("Initializing Hovergames Gazebo Interface...");

    // Subscriber for sensor data
    state_sub_ = nh.subscribe(config_->robot_state_topic_, 1, &HovergamesGazeboInterface::StateCallBack, this);

    // Publisher for vehicle command
    command_pub_ = nh.advertise<mav_msgs::RollPitchYawrateThrust>(config_->cmd_, 1);

    // Service servers for running node
    enable_control_server_ = nh.advertiseService("px4_ext_cont_enable", &HovergamesGazeboInterface::EnableControlCallback, this);
    disable_control_server_ = nh.advertiseService("px4_ext_cont_disable", &HovergamesGazeboInterface::DisableControlCallback, this);

    // Service client for transferring control to PX4 control interface
    mission_finished_client_ = nh.serviceClient<std_srvs::Trigger>("px4_mission_finished_ext_cont");

    LMPCC_WARN_ALWAYS("Hovergames Gazebo Interface initialized");
}

void HovergamesGazeboInterface::ActuateNow()
{
    // Publish the command
    control_msg_.header.stamp = ros::Time::now();
    command_pub_.publish(control_msg_);
}

void HovergamesGazeboInterface::Actuate()
{
    // Fill command message
    control_msg_.header.frame_id = config_->robot_base_link_;

    control_msg_.roll = solver_interface_ptr_->phi_c();
    control_msg_.pitch = solver_interface_ptr_->theta_c();
    control_msg_.yaw_rate = solver_interface_ptr_->dpsi_c();
    control_msg_.thrust.z = (solver_interface_ptr_->thrust_c() - 9.81) / a_t_ + t_h_; // TODO: set this value correctly!
}

void HovergamesGazeboInterface::ActuateBrake(double deceleration)
{
    // Ignore deceleration value on the drone

    // Fill command message
    control_msg_.header.frame_id = config_->robot_base_link_;

    control_msg_.roll = 0;
    control_msg_.pitch = 0;
    control_msg_.yaw_rate = 0;
    control_msg_.thrust.z = 0.674;
}

void HovergamesGazeboInterface::Reset()
{
    // No reset from LMPCC supported: use drone_toolbox functionality to reset
}

/* CALLBACKS */
bool HovergamesGazeboInterface::EnableControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    LMPCC_INFO("EnableControlCallback called: enabling MPCC::ControlLoop() by starting timer.");
    controller_->timer_.start();
    controller_->timer_running_ = true;
    res.success = true;
    res.message = "Enabled MPCC control loop.";
    return true;
}

bool HovergamesGazeboInterface::DisableControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    LMPCC_INFO("DisableControlCallback called: disabling MPCC::ControlLoop() by stopping timer.");
    controller_->timer_.stop();
    controller_->timer_running_ = false;
    res.success = true;
    res.message = "Disabled MPCC control loop.";
    return true;
}

void HovergamesGazeboInterface::StateCallBack(const nav_msgs::Odometry &msg)
{
    PROFILE_FUNCTION(); // To track when the state is received
    LMPCC_WARN("State Callback");

    // Calculate Euler angles from received quaternion
    tf2::Quaternion q_tf;
    tf2::convert(msg.pose.pose.orientation, q_tf);
    tf2Scalar roll, pitch, yaw;
    tf2::Matrix3x3(q_tf).getEulerYPR(yaw, pitch, roll);

    // Update the frame
    config_->target_frame_ = msg.header.frame_id;

    // Update the states
    solver_interface_ptr_->State().set_x(msg.pose.pose.position.x);
    solver_interface_ptr_->State().set_y(msg.pose.pose.position.y);
    solver_interface_ptr_->State().set_psi(msg.pose.pose.position.z);
    solver_interface_ptr_->State().set_vx(msg.twist.twist.linear.x);
    solver_interface_ptr_->State().set_vy(msg.twist.twist.linear.y);
    solver_interface_ptr_->State().set_vz(msg.twist.twist.linear.z);
    solver_interface_ptr_->State().set_phi(roll);
    solver_interface_ptr_->State().set_theta(pitch);
    solver_interface_ptr_->State().set_psi(yaw);

    if (config_->debug_output_)
        solver_interface_ptr_->State().print();

    controller_->OnStateReceived();
}
