/**
 * @file interface.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Base class for an interface to a simulation environment or a real-world robot
 * Allows to customize the input/output data messages without changing the controller code.
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef INTERFACE_H
#define INTERFACE_H

#include <lmpcc/types.h>

#include <geometry_msgs/Pose.h>

#include <std_msgs/Float32.h>

#include <lmpcc_msgs/follower_state.h>
#include <lmpcc_msgs/follower.h>

#include <lmpcc_msgs/gaussian.h>
#include <lmpcc_msgs/obstacle_array.h>
#include <lmpcc_msgs/obstacle_gmm.h>

#include <lmpcc_msgs/halfspace.h>
#include <lmpcc_msgs/halfspace_array.h>
#include <lmpcc_msgs/observation_partitioning.h>

// #include <pedsim_msgs/AgentStates.h>
// #include <pedsim_msgs/AgentState.h>

#include <ros_tools/profiling.h>
// #include <lmpcc/lmpcc_configuration.h>
// #include <lmpcc/PredictiveControllerConfig.h>
// #include "lmpcc/dynamic_obstacle.h"
// #include "lmpcc_solver/SolverInclude.h"

class SolverInterface;
class Controller;

class Interface
{

public:
    /**
     * @brief Base interface constructor, initializes variables that are always useful
     *
     * @param nh nodehandle
     * @param controller pointer to the controller
     * @param config parameters
     * @param solver_interface_ptr solver interface
     */
    Interface(ros::NodeHandle &nh, Controller *controller, predictive_configuration *config, SolverInterface *solver_interface_ptr);

    // Virtual in case any interface wants to extend it
    virtual ~Interface(){};

protected:
    // Configuration
    predictive_configuration *config_;

    // Necessary for callbacks
    Controller *controller_;

    /**
     * @brief Plot the obstacles as in dynamic_obstacles of data_
     *
     * @param markers The ros marker publisher to publish the obstacles in
     * @param plot_models Should we plot 3D pedestrian models at the location of the obstacles? (Default = False)
     * @param publish Should this function also publish all markers in the topic? (Default = True)
     */
    void PlotObstacles(RosTools::ROSMarkerPublisher *markers, bool plot_models = false, bool publish = true);

public:
    /**
     * @brief Ensure that the number of obstacles match the indended size (config_->max_obstacles_)
     */
    void PostProcessObstacles(bool (*reject_function)(const Eigen::Vector2d &vehicle_pos, const Eigen::Vector2d &obstacle_pos) = nullptr);

    /** @brief: Connects LMPCC with partitioning of scenarios */
    virtual void PartitionCallback(const lmpcc_msgs::observation_partitioning &msg);

    /** @brief Connects LMPCC with the roadmap */
    virtual void RoadmapWaypointsCallback(const nav_msgs::Path &msg);

    /** @brief Connects LMPCC with Pedsim ROS Pedestrian Simulator */
    // virtual void PedsimObstacleCallBack(const pedsim_msgs::AgentStates &msg);

    /** @brief Connect LMPCC with the Follower */
    virtual lmpcc_msgs::follower SendTrajectoryToFollower(SolverInterface *solver_interface);

    virtual void GoalCallback(const geometry_msgs::PoseStamped &msg);

    SolverInterface *solver_interface_ptr_;

    RealTimeData data_;

    std::unique_ptr<Monitor> monitor_;
    std::vector<RosTools::SignalPublisher> signal_publishers_; /* Mainly intended for publishing control signals to be visualized by jsk_rviz_plugins */

    std::unique_ptr<RosTools::ROSMarkerPublisher> ros_markers_;

    // Pose to reset to in simulation
    geometry_msgs::Pose reset_pose_;

    bool collision_detected_;

    ros::Time t_state_received_;

    double reference_velocity_;
    double contouring_weight_;
    bool external_velocity_set_;

    lmpcc_msgs::obstacle_array prediction_msg_;

    /** Functions that must be defined for ALL interfaces */
    /**
     * @brief Actuate the system (currently only stores actuation until ActuateNow is called!)
     * @see ActuateNow
     */
    virtual void Actuate() = 0;

    /**
     * @brief Actuate a braking acceleration or force (used for safe behavior if the controller fails)
     *
     * @param deceleration Deceleration to brake with
     */
    virtual void ActuateBrake(double deceleration) = 0;

    /**
     * @brief Forward the actuation commands to the robot
     *
     */
    virtual void ActuateNow(){};

    /**
     * @brief If the solver could not find a solution, insert an emergency plan into the solver and actuate this plan.
     *
     * @param deceleration The deceleration to plan for.
     */
    virtual void DeployEmergencyStrategy(double deceleration)
    {
        EmergencyStrategyConstantBraking(deceleration);
    }; // Default is constant braking without steering

    /**
     * @brief Reset to a given position
     *
     * @param position_after_reset The position to reset to
     */
    virtual void Reset(const geometry_msgs::Pose &position_after_reset) { Reset(); };

    /**
     * @brief Define actions to take when resetting (invoked from MPCC)
     */
    virtual void Reset() = 0;

protected:
    // Emergency plans
    /** @brief Constant deceleration without steering*/
    virtual void EmergencyStrategyConstantBraking(double deceleration);
};

#endif