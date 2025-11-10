#ifndef ROBOAT_INTERFACE_H
#define ROBOAT_INTERFACE_H

#include "interfaces/interface.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64MultiArray.h>
// TF
#include <tf2_ros/transform_broadcaster.h>
#include <lmpcc_msgs/Control.h>
#include <lmpcc_msgs/Force.h>
#include <robot_localization/SetPose.h>

#include <ros_tools/helpers.h>
#include <lmpcc_msgs/halfspace.h>
#include <lmpcc_msgs/halfspace_array.h>

class RoboatInterface : public Interface
{
public:
    RoboatInterface(ros::NodeHandle &nh, MPCC *controller, predictive_configuration *config, SolverInterface *solver_interface_ptr);

private:
    ros::Subscriber state_sub_, velocity_sub_;
    ros::Subscriber obstacle_sub_, waypoints_sub_, reset_sub_, linearconstraints_sub_;
    ros::Publisher command_pub_;
    ros::Publisher reset_roboat_pub_;
    ros::Publisher obstacles_pub_;

    // Publish visualization of repulsive costs for obstacles
    ros::Publisher repulsive_ellipsoids_pub, repulsive_regulations_pub, repulsive_rightofway_pub;

    // Service clients
    ros::ServiceClient reset_simulation_client_, reset_ekf_client_;

    // And reset messages
    std_srvs::Empty reset_msg_;
    robot_localization::SetPose reset_pose_msg_;

    tf2_ros::TransformBroadcaster path_pose_pub_;

    bool goal_set_;
    Eigen::Vector2d goal_location_;

    SolverInterface *solver_interface_ptr_;

    void OrderObstacles(lmpcc_msgs::lmpcc_obstacle_array &ellipses);

public:
    virtual void Actuate() override;
    virtual void ActuateBrake(double deceleration) override;
    virtual void Reset(const geometry_msgs::Pose &position_after_reset) override;
    virtual void Reset() override;

    void ResetCallback(const std_msgs::Empty &msg);

    void StateCallBack(const nav_msgs::Odometry &msg);

    void ObstacleCallBack(const lmpcc_msgs::lmpcc_obstacle_array &received_obstacles);

    void WaypointsCallback(const nav_msgs::Path &msg);

    void plotObstacles(void);

    void LinearConstraintsCallback(lmpcc_msgs::halfspace_array linearconstraints_msg);

    void determinePriorityVessels(const lmpcc_msgs::lmpcc_obstacle_array &obstacles);

    void visualizeRepulsiveEllipsoidCosts();

    void visualizeRepulsiveRegulationCosts();

    void visualizeRepulsiveRightofWay();

    std::map<int, bool> priority_vessels_;
};

#endif