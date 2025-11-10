/**
 * @file prius_interface.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Interface for controlling the IV demonstrator vehicle (Modified Toyota Prius)
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef PRIUS_INTERFACE_H
#define PRIUS_INTERFACE_H

#include "interfaces/interface.h"

#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <lmpcc_msgs/Control.h>
#include <roadmap_msgs/RoadPolylineArray.h>

#include <ros_tools/helpers.h>
#include <scenario/gaussian_sampler.h>

class MPCC;


class PriusInterface : public Interface
{

public:
    PriusInterface(ros::NodeHandle &nh, MPCC *controller, predictive_configuration *config, SolverInterface *solver_interface_ptr);

private:
    std::unique_ptr<RosTools::ROSMarkerPublisher> obstacle_markers_;

    ros::Subscriber state_sub_, steering_sub_, acceleration_sub_;
    ros::Subscriber obstacle_prediction_sub_, reference_path_sub_;
    ros::Publisher command_pub_;
    ros::Publisher vehicle_speed_pub_;
    ros::Publisher reset_simulation_pub_;
    ros::Publisher plot_throttle_pub_;
    ros::Publisher dt_pub_, N_pub_, hz_pub_;

    lmpcc_msgs::obstacle_array sorted_obstacles_;
    lmpcc_msgs::obstacle_gmm dummy_obstacle_;

    lmpcc_msgs::obstacle_array obstacle_msg_;

    std::vector<int> obstacle_indices_;
    std::vector<double> obstacle_distances_;

    bool reference_received_;
    bool delayed_ = true;

public:
    virtual void Actuate() override;
    virtual void ActuateBrake(double deceleration) override;
    virtual void Reset();

    void StateCallBack(const nav_msgs::Odometry::ConstPtr &msg);
    void AccelerationCallback(const geometry_msgs::AccelWithCovarianceStamped &msg);
    void SteeringAngleCallback(const sensor_msgs::JointState &msg);

    /**
     * @brief Callback for obstacle predictions.
     * We assume that obstacles are not ordered and that the size may not match max_obstacles.
     *
     * @param received_obstacles
     */
    void ObstacleCallBack(const lmpcc_msgs::obstacle_array &received_obstacles);
    void PreProcessObstacles();
    void SortObstacles();

    /** @brief Plot all obstacles (processed and received) */
    void PlotAllObstacles();
    void WaypointsCallback(const nav_msgs::Path &msg);

    /** @brief Create a list of obstacles internally (without receiving data) */
    void DebugInsertFakeObstacles();
};

#endif
