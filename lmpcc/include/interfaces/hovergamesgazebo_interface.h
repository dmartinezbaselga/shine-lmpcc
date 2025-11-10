/**
 * @file hovergamesgazebo_interface.h
 * @author Dennis Benders (d.benders@tudelft.nl)
 * @brief Interface with the Hovergames drone simulator in Gazebo
 * @version 0.1
 * @date 2022-05-12
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef HOVERGAMES_GAZEBO_INTERFACE_H
#define HOVERGAMES_GAZEBO_INTERFACE_H

#include "interfaces/interface.h"

#include <mav_msgs/RollPitchYawrateThrust.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ros_tools/helpers.h>

class HovergamesGazeboInterface : public Interface
{

public:
    HovergamesGazeboInterface(ros::NodeHandle &nh, MPCC *controller, predictive_configuration *config, SolverInterface *solver_interface_ptr);

public:
    virtual void ActuateNow() override;
    virtual void Actuate() override;
    virtual void ActuateBrake(double deceleration) override;
    virtual void Reset();

    bool EnableControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool DisableControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    void StateCallBack(const nav_msgs::Odometry &msg);

private:
    // Publishers and subscribers
    ros::Subscriber state_sub_;
    ros::Publisher command_pub_;

    // Service server
    ros::ServiceServer enable_control_server_, disable_control_server_;

    // Service client
    ros::ServiceClient mission_finished_client_;

    // Command message
    mav_msgs::RollPitchYawrateThrust control_msg_;

    // Reset message
    std_srvs::Trigger mission_finished_srv_;

    // Thrust command conversion
    double a_t_ = 22.629;
    double t_h_ = 0.674;
};

#endif // HOVERGAMES_GAZEBO_INTERFACE_H
