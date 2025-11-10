/**
 * @file jackalsocnavbench_interface.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Interface with the Jackal socnavbench in Gazebo (https://www.clearpathrobotics.com/assets/guides/melodic/jackal/simulation.html)
 * @note Some modifications are made to this environment
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef JACKAL_SOCNAVBENCH_INTERFACE_H
#define JACKAL_SOCNAVBENCH_INTERFACE_H

#include <interfaces/interface.h>

#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Path.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Time.h>

#include <tf2_ros/transform_broadcaster.h>
#include <robot_localization/SetPose.h>

#include <derived_object_msgs/ObjectArray.h>
#include <derived_object_msgs/Object.h>

#include <lmpcc_msgs/obstacle_array.h>
#include <lmpcc_msgs/obstacle_gmm.h>
#include <lmpcc_msgs/gaussian.h>

#include <lmpcc_msgs/SolveMPC.h>
#include <lmpcc_msgs/InfoMPC.h>

class MPCC;

class JackalSocnavbenchInterface : public Interface
{

public:
    JackalSocnavbenchInterface(ros::NodeHandle &nh, MPCC *controller, predictive_configuration *config, SolverInterface *solver_interface_ptr);

public:
    virtual void ActuateNow() override;
    virtual void Actuate() override;
    virtual void ActuateBrake(double deceleration) override;
    void Reset() override;

    // void PartitionCallback(const lmpcc_msgs::observation_partitioning &msg);

    void PlotAllObstacles();

private:
    double dt = 0.2, N = 30;
    ros::NodeHandle nh2;
    ros::ServiceServer shine_srv_;
    bool handleShine(lmpcc_msgs::SolveMPC::Request& req, lmpcc_msgs::SolveMPC::Response& res);
    void updateData(const lmpcc_msgs::SolveMPC::Request& req);
    bool rotateToGoal(double x, double y, double theta, double goal_x, double goal_y, double& v, double& w);

    ros::Subscriber state_sub_, velocity_sub_;
    ros::Subscriber obstacle_sub_, prediction_sub_, trajectory_prediction_sub_, waypoints_sub_, reset_sub_, partition_sub_, goal_sub_;
    ros::Subscriber pedsim_sub_;
    ros::Subscriber collision_sub_;

    ros::Publisher command_pub_, time_pub_;
    ros::Publisher reset_simulation_pub_;
    ros::Publisher dt_pub_, N_pub_, hz_pub_;
    ros::Publisher plot_throttle_pub_, plot_steering_pub_, plot_velocity_pub_;

    std::unique_ptr<RosTools::ROSMarkerPublisher> obstacle_markers_;
    std::unique_ptr<RosTools::ROSMarkerPublisher> road_markers_;

    // Keep track of the number of experiments
    std::unique_ptr<RosTools::SimulationTool> simulation_tool_;

    // Service clients
    ros::ServiceClient reset_simulation_client_, reset_ekf_client_;

    // And reset messages
    std_srvs::Empty reset_msg_;
    robot_localization::SetPose reset_pose_msg_;

    // pedsim_msgs::TrackedPersons obstacle_msg_;
    derived_object_msgs::ObjectArray obstacle_msg_;
    tf2_ros::TransformBroadcaster path_pose_pub_;

    geometry_msgs::Twist control_msg_;

    ros::Time t_last_reset_, t_last_tf_;

    // void CreateObstacleList();
    void PlotRoad();
};

#endif