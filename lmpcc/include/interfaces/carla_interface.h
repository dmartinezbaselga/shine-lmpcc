/**
 * @file carla_interface.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Interface for the Carla simulator (https://carla.readthedocs.io/en/latest/python_api/) with ROS Bridge (https://github.com/carla-simulator/ros-bridge)
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef CARLA_INTERFACE_H
#define CARLA_INTERFACE_H

#include <cmath>

#include "interfaces/interface.h"

#include <scenario/gaussian_sampler.h>

#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <carla_msgs/CarlaEgoVehicleInfo.h>
#include <carla_msgs/CarlaStatus.h>
#include <carla_msgs/CarlaCollisionEvent.h>

#include <lmpcc/LMPCCReset.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <carla_ackermann_control/EgoVehicleControlInfo.h>
#include <carla_msgs/CarlaStatus.h>
#include <carla_msgs/CarlaCollisionEvent.h>

#include <derived_object_msgs/ObjectArray.h>
#include <tf2_ros/transform_broadcaster.h>

#include <lmpcc_msgs/observation_partitioning.h>

#include <lmpcc_msgs/lmpcc_obstacle_array.h>
#include <lmpcc_msgs/lmpcc_obstacle.h>

#include <ros_tools/helpers.h>
#include <ros_tools/data_saver.h>

class CarlaInterface : public Interface
{

public:
    CarlaInterface(ros::NodeHandle &nh, MPCC *controller, predictive_configuration *config, SolverInterface *solver_interface_ptr);

    ~CarlaInterface()
    {
        data_saver_.SaveData("low_level_control");
    }

private:
private:
    RosTools::DataSaver data_saver_;

    double last_velocity_time_, time_;
    double last_velocity_;

    std::unique_ptr<RosTools::ROSMarkerPublisher> ros_markers_, ros_all_obstacle_markers_, ros_predicted_obstacle_markers_;

    ros::Subscriber partition_sub_;
    ros::Subscriber state_sub_, steering_sub_, acceleration_sub_;
    ros::Subscriber obstacle_sub_, obstacle_prediction_sub_, obstacle_sample_sub_, waypoints_sub_, vehicle_info_sub_, obstacle_prediction_sub_hardcoded_;
    ros::Subscriber reset_pose_sub_, carla_status_sub_;
    // ros::Subscriber carla_status_sub_; // For debugging purposes
    ros::Subscriber reference_velocity_sub_, contouring_weight_sub_;
    ros::Subscriber halfspace_sub_;
    ros::Subscriber goal_reached_sub;
    // For debugging
    ros::Subscriber ackermann_info_sub_;
    ros::Subscriber collision_sub_;
    ros::Publisher command_pub_;
    ros::Publisher goal_pub_;
    ros::Publisher reset_carla_pub_;
    ros::Publisher reset_pub_;
    ros::Publisher obstacles_pub_;
    ros::Publisher sample_size_pub_;
    ros::Publisher dt_pub_, N_pub_, hz_pub_;

    ackermann_msgs::AckermannDrive command_msg_;
    geometry_msgs::Twist twist_msg_;

    carla_msgs::CarlaEgoVehicleInfo ego_vehicle_info_;

    ros::Time last_transform_time_;
    tf2_ros::TransformBroadcaster state_pub_;
    RosTools::DataSaver time_taken_to_finish_;
    bool goal_set_;
    Eigen::Vector2d goal_location_;
    int current_obstacle_hit = 0;

    bool delayed_ = true;

    lmpcc_msgs::halfspace dummy_halfspace_;
    lmpcc_msgs::obstacle_array last_predictions_;
    bool received_external_prediction_;

    derived_object_msgs::ObjectArray sorted_obstacles_;
    derived_object_msgs::Object dummy_obstacle_;
    std::vector<int> obstacle_indices_;
    std::vector<double> obstacle_distances_;

    bool transformPose(const std::string &from, const std::string &to, geometry_msgs::Pose &pose);
    void computeEgoDiscs();
    double r_discs_;

public:
    bool collision_detected_;

    virtual void Actuate() override;
    virtual void ActuateBrake(double deceleration) override;

    virtual void Reset(const geometry_msgs::Pose &position_after_reset) override;
    virtual void Reset();

    // Reset from rviz initial pose value
    void ResetCallback(const geometry_msgs::PoseWithCovarianceStamped &initial_pose);
    void carlaCollisionCallback(const carla_msgs::CarlaCollisionEvent &msg);
    void StateCallBack(const nav_msgs::Odometry::ConstPtr &msg);
    void AccelerationCallback(const geometry_msgs::AccelWithCovarianceStamped &msg);

    // Callback for the static map
    void halfspaceCallback(const lmpcc_msgs::halfspace_array &msg);

    void AckermannCallback(const carla_ackermann_control::EgoVehicleControlInfo &msg);

    void DeployEmergencyStrategy(double deceleration) override;

    void ObstacleCallBack(const derived_object_msgs::ObjectArray &received_obstacles);
    // void ObstaclePredictionsCallback(const lmpcc_msgs::obstacle_array &msg);
    // void ObstacleSampleCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void ObstacleTrajectoryPredictionsCallback(const lmpcc_msgs::obstacle_array &msg);
    static bool ObstacleRejectFunction(const Eigen::Vector2d &vehicle_pos, const Eigen::Vector2d &obstacle_pos);

    void VehicleInfoCallback(const carla_msgs::CarlaEgoVehicleInfo &msg);
    void carlaGoalPositionCallback(const geometry_msgs::PoseStamped &msg);
    void carlaStatusCallback(const carla_msgs::CarlaStatus &msg);

    void referenceVelocityCallback(const std_msgs::Float64 &msg);
    void contouringWeightCallback(const std_msgs::Float64 &msg);

    void PlotPredictedObstacles();
    void PlotReceivedObstacles();

    void SortObstacles();
    void SelectObstacles(const std::vector<int> &indices);

    void PlotStraightRoad(void);
};

#endif