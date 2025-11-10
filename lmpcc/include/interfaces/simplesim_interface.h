/**
 * @file simple_sim_interface.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl), Victor van der Drift (vvjvanderdrift@gmail.com)
 * @brief Interface for the simple_sim python simulator (https://github.com/R2CLab/simple_sim)
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef SIMPLE_SIM_INTERFACE_H
#define SIMPLE_SIM_INTERFACE_H

#include "interfaces/interface.h"

#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <lmpcc_msgs/Control.h>
#include <nav_msgs/Odometry.h>
#include <roadmap_msgs/RoadPolylineArray.h>
#include <sensor_msgs/JointState.h>

#include <ros_tools/helpers.h>

class MPCC;

class SimpleSimInterface : public Interface
{

public:
  SimpleSimInterface(ros::NodeHandle &nh, Controller *controller, predictive_configuration *config, SolverInterface *solver_interface_ptr);

private:
  std::unique_ptr<RosTools::ROSMarkerPublisher> obstacle_markers_;

  ros::Subscriber state_sub_, steering_sub_, acceleration_sub_, tire_forces_Fyf_sub_, tire_forces_Fyr_sub_;
  ros::Subscriber obstacle_prediction_sub_, reference_path_sub_;
  ros::Publisher command_pub_;
  ros::Publisher vehicle_speed_pub_;
  ros::Publisher reset_simulation_pub_;
  ros::Publisher reset_simple_sim_pub_;
  ros::Publisher plot_throttle_pub_;
  ros::Publisher dt_pub_, N_pub_, hz_pub_;
  ros::Publisher follower_pub_;

  lmpcc_msgs::obstacle_array sorted_obstacles_;
  lmpcc_msgs::obstacle_gmm dummy_obstacle_;

  lmpcc_msgs::obstacle_array obstacle_msg_;

  std::vector<int> obstacle_indices_;
  std::vector<double> obstacle_distances_;
  std::unique_ptr<RosTools::SimulationTool> simulation_tool_;
  bool reference_received_;
  bool delayed_ = true;

  std::string tire_forces_Fyf_topic_ = "/value/Fyf";
  std::string tire_forces_Fyr_topic_ = "/value/Fyr";

  double Fyf_, Fyr_; /* Save tire forces from simulator*/

public:
  virtual void Actuate() override;
  virtual void ActuateFollower(double control_v, double control_delta);
  virtual void ActuateBrake(double deceleration) override;
  virtual void Reset();

  void StateCallBack(const nav_msgs::Odometry::ConstPtr &msg);
  void AccelerationCallback(const geometry_msgs::AccelWithCovarianceStamped &msg);
  void SteeringAngleCallback(const sensor_msgs::JointState &msg);

  /* Callback and Fetters for saving tire forces from simulation */
  void TireForcesFyfCallback(const std_msgs::Float32 &msg);
  void TireForcesFyrCallback(const std_msgs::Float32 &msg);
  double getTireForcesFyf();
  double getTireForcesFyr();

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
