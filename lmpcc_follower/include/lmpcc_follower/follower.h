#ifndef __FOLLOWER_H__
#define __FOLLOWER_H__

#include <ros/ros.h>

#include <lmpcc_msgs/follower.h>
#include <lmpcc_msgs/follower_state.h>

// RQT Reconfigure
#include <dynamic_reconfigure/server.h>
#include <lmpcc_follower/FollowerConfig.h>

#include "generated_cpp/InterfaceDefinition.h"
#include "interfaces/interface.h"
#include "interfaces/simplesim_interface.h"

#include "lmpcc/lmpcc_configuration.h"
#include "lmpcc/types.h"
#include "lmpcc_tools/helpers.h"

#include <lmpcc_follower/configuration.h>
#include <lmpcc_follower/pid_stanley.h>

#define INTERFACE_CLASS SimpleSimInterface /** To debug! */

class Follower : public Controller
{
public:
  Follower();

  virtual ~Follower(){};

public:
  bool Initialize();

  // void LoadPlannerSolverInfo(SolverInterface *planner_solver_interface, const double planner_optimization_duration);

  void Update(const ros::TimerEvent &event);

  void CalculatePID_StanleyControls();
  void SavePID_StanleyData();
  lmpcc_msgs::follower_state SplinePointReference();

  void OnObstaclesReceived() override{};
  void OnStateReceived() override{};
  void OnWaypointsReceived() override{};
  void OnWeightsReceived() override{};
  void OnReset() override;

  void PublishReferencePath(const lmpcc_msgs::follower_state &reference_point);
  void ExportData();
  void SaveExportData();

  void PlannerCallback(const lmpcc_msgs::follower &msg);

  DataSaver data_saver_; /* Used to store runtime data */

  std::unique_ptr<PID_Stanley> pid_stanley_;

private:
  ros::NodeHandle nh_;

  std::unique_ptr<FollowerConfig> config_;                 // Follower configuration
  std::unique_ptr<predictive_configuration> lmpcc_config_; // Configuration of LMPCC!

  ros::Subscriber planner_sub_; // Connects this node to the lmpcc
  lmpcc_msgs::follower follower_msg_;

  int experiment_counter_; /* Count experiments */

  int follower_counter_ = -1;
  ros::Timer timer_follower_; /** Timer to run the follower update*/
  bool timer_running_;

  double planner_optimization_duration_ = 0.; // Duration taken by planner optimization, should be larger than 0. so initialized as -1 as a check.

  double planner_prediction_integrator_stepsize_ = 0.2; // Set in the planner optimization prius_settings.py
  unsigned int planner_horizon_size_ = 3;

  std::unique_ptr<SimpleSimInterface> system_interface_;
  std::unique_ptr<SolverInterface> solver_interface_; // To solve the follower MPC problem
  // std::unique_ptr<SolverInterface> planner_solver_interface_; // Currently only for saving the planner info used as reference for the follower

  std::unique_ptr<tk::spline> spline_x_, spline_y_, spline_psi_;
  std::unique_ptr<ROSMarkerPublisher> ros_markers_reference_path_, ros_markers_reference_arrows_;

  ros::Time time_of_loading_trajectory_ = ros::Time::now();
  ros::Time time_of_setting_spline = ros::Time::now();
};

#endif // __FOLLOWER_H__