/**
 * @file lmpcc_controller.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Main controller class
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef LMPCC_DEMAND_H
#define LMPCC_DEMAND_H

// Configuration
#include <lmpcc/lmpcc_configuration.h>

// Helpers / util / types
#include <lmpcc/experiment-util.h>
// #include <lmpcc/types.h>

// // Solver generated
#include <lmpcc_solver/InterfaceDefinition.h>
#include <lmpcc_solver/SolverInclude.h>

// // ros includes
// #include <geometry_msgs/PoseArray.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <pluginlib/class_loader.h>
// #include <std_msgs/Bool.h>
// #include <std_msgs/Float32.h>
// #include <std_msgs/Float64.h>
// #include <std_msgs/Float64MultiArray.h>

// // yaml parsing
// #include <yaml-cpp/yaml.h>

// Dynamic Reconfigure server
#include <dynamic_reconfigure/server.h>
#include <lmpcc/PredictiveControllerConfig.h>

// // actions, srvs, msgs
// #include <actionlib/client/simple_action_client.h>
// #include <actionlib/server/simple_action_server.h>
// #include <nav_msgs/Odometry.h>
// #include <nav_msgs/Path.h>

// // lmpcc messages
// #include <lmpcc_msgs/Control.h>
// #include <lmpcc_msgs/halfspace.h>
// #include <lmpcc_msgs/halfspace_array.h>
// #include <lmpcc_msgs/lmpcc_obstacle.h>
// #include <lmpcc_msgs/lmpcc_obstacle_array.h>

// // Joint states
// #include <sensor_msgs/JointState.h>

// Visuals
// #include <ros_tools/data_saver.h>
#include <ros_tools/helpers.h>
#include <ros_tools/ros_visuals.h>

#include <ros/ros.h>
// // reset msgs
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/PoseWithCovariance.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <std_srvs/Empty.h>

// #include <algorithm>
// #include <boost/filesystem.hpp>
// #include <boost/scoped_ptr.hpp>
// #include <boost/shared_ptr.hpp>
// #include <boost/thread/mutex.hpp>
// #include <cmath>
// #include <fstream>
// #include <iomanip>
// #include <iostream>
// #include <limits>
// #include <map>
// #include <math.h>
#include <memory>
// #include <numeric>
// #include <ros/package.h>
// #include <ros/ros.h>
// #include <sstream>
// #include <stdexcept>
// #include <string>
// #include <tf/tf.h>
// #include <tf/transform_listener.h>
// #include <thread>
#include <vector>
#include "lmpcc/lmpcc_controller.h"


/**
 * @brief Enumerator used for tracking the control status
 */
// class ControllerStatus;

class Interface;

/**
 * @brief Controller class for Model Predictive Contouring Control.
 */
class MPCCOnDemand : public MPCC
{
public:
  MPCCOnDemand();
  bool initialize___();
  bool initializeAndStopTimer();
  void RunOnDemand() override;

  void OnReset() override;


  ~MPCCOnDemand();

  // void OnReset() override;             /**   * @brief Handles a reset of the controller   */
  // void OnObstaclesReceived() override; /**   @brief Called when obstacle data is received, forwards data if necessary   */
  // void OnStateReceived() override;     /**   @brief Called when the state is received, checks for jumps in the state to reinitialize the reference path   */
  // void OnWaypointsReceived() override; /**   @brief Called when waypoints are received, reinitalizes the reference path   */
  // void OnWeightsReceived() override;   /**   @brief Called when external weights are received, updates the reconfigure window and local variables   */
  // void OnOtherDataReceived(std::string &&data_name) override;

protected:

  // Responsible for
  // - Updating real-time data
  // - Constructing constraints and inserting solver parameters
  // - Solving the optimization
  // - Fault handling, and
  // - Actuation
  void ControlLoop();                         // Main control function, executed with the control frequency
  // void runNode(const ros::TimerEvent &event) override; // Runs <ControlLoop>"()"

  // void SaveControllerData() override; // Store control data for saving later

  // void printInfoAtNoSuccess(int &exit_code) const override; // Print the error and exit code in case the optimization is not successful
  // void publishCurrentCollisionSpace(void) override;         // Publish the collision space of the robot for the received state
  // void publishPlan(void) override;                          // Visualize the robot collision space along the plan.
  // void publishInitialPlan(void) override;                   // Visualize the initial guess before optimization
};

#endif
