/**
 * @file jackalsimulator_interface.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Interface with the Jackal simulator in Gazebo (https://www.clearpathrobotics.com/assets/guides/melodic/jackal/simulation.html)
 * @note Some modifications are made to this environment
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef JACKAL_INTERFACE_H
#define JACKAL_INTERFACE_H

#include "interfaces/interface.h"

#include <std_srvs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include <ros_tools/helpers.h>

class JackalInterface : public Interface
{

public:
    JackalInterface(ros::NodeHandle &nh, Controller *controller, predictive_configuration *config, SolverInterface *solver_interface_ptr);

public:
    virtual void ActuateNow() override;
    virtual void Actuate() override;
    virtual void ActuateBrake(double deceleration) override;
    virtual void Reset();

    void ResetCallback(const std_msgs::Empty &msg);

    void StateOptitrackCallBack(const geometry_msgs::PoseStamped &msg);
    void OptitrackObstacleCallback(const std_msgs::Float64MultiArray &received_obstacles);
    void OptitrackPredictionCallback(const std_msgs::Float64MultiArray &received_obstacles);

    void BluetoothCallBack(const sensor_msgs::Joy &msg);

    void GoalCallback(const geometry_msgs::PoseStamped &msg);

    /** @deprecated */
    void StateCallBack(const nav_msgs::Odometry &msg);
    void VelocityCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void ObstaclePredictionsCallback(const lmpcc_msgs::obstacle_array &msg);
    void ObstacleTrajectoryPredictionsCallback(const lmpcc_msgs::obstacle_array &msg);

    geometry_msgs::Twist control_msg_;
    bool enable_output_;

private:
    ros::Subscriber state_sub_, velocity_sub_, bluetooth_sub_;
    ros::Subscriber trajectory_prediction_sub_, reset_sub_;
    ros::Subscriber waypoints_sub_, goal_sub_;
    std::vector<ros::Subscriber> prediction_subs_;
    std::vector<ros::Subscriber> obstacle_subs_;

    ros::Publisher command_pub_;
    ros::Publisher reset_simulation_pub_;
    ros::Publisher reverse_roadmap_pub_;

    // Keep track of the number of experiments
    std::unique_ptr<RosTools::SimulationTool> simulation_tool_;

    // Service clients
    ros::ServiceClient reset_simulation_client_, reset_ekf_client_;

    // And reset messages
    std_srvs::Empty reset_msg_;

    tf2_ros::Buffer tf_buffer_;
    tf::TransformListener listener;

    double last_x_, last_y_;
    ros::Time last_state_time_, last_reset_time_;

    lmpcc_msgs::halfspace dummy_halfspace_;

    int first_obstacle_id_ = -1;

    bool rotating_towards_goal_ = false;
    void RotateTowardsGoal();

    void CreateObstacleList();
};

#endif
