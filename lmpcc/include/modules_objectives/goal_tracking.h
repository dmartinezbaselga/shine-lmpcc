/**
 * @file reference_path.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Manages the reference path of a contouring based MPC
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef GOAL_TRACKING_H
#define GOAL_TRACKING_H

#include <lmpcc/types.h>

#include <ros_tools/ros_visuals.h>

#include <Eigen/Dense>

class predictive_configuration;
class SolverInterface;
class VehicleRegion;
// Whens earching for the closest point on the path, this variable indicates the distance that the algorithm searches
// behind the current spline point.
#define MAX_STEP_BACK_TOLERANCE 0.1f

class GoalTracking : public ControllerModule
{
public:
  /**
   * @brief Initialize the reference path, reading the path from file.
   *
   * @param nh nodehandle
   * @param config parameters
   */
  GoalTracking(ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle);

public:
  bool goal_reached_ = false;
  bool first_run_ = true;

  bool ReadyForControl(SolverInterface *solver_interface, const RealTimeData &data) override;

  /**
   * @brief Checks if the end of the reference path was reached
   *
   * @param solver_interface
   * @param data
   * @return true If it reached the end
   * @return false otherwise
   */
  bool ObjectiveReached(SolverInterface *solver_interface, const RealTimeData &data) override;

  /**
   * @brief Update the reference path at the start of a control computation
   *
   * @param solver_interface The solver
   * @param data Real-time data from the interface
   */
  void Update(SolverInterface *solver_interface, RealTimeData &data) override;

  void OnReset(SolverInterface *solver_interface) override;

  void OnDataReceived(SolverInterface *solver_interface, RealTimeData &data, std::string &&data_name) override;

  /**
   * @brief Load parameters of the splines into the solver
   *
   * @param solver_interface solver interface (pointer)
   * @param N_iter time step to load for
   * @param param_idx parameter index (increased internally)
   */
  void SetParameters(SolverInterface *solver_interface, const RealTimeData &data, int N_iter, int &param_idx) override;

  /**
   * @brief Visualize the reference path
   *
   */
  void Visualize() override;

private:
  /* Visuals */
  std::unique_ptr<RosTools::ROSMarkerPublisher> ros_markers_;

  bool goal_received_;
  bool goal_not_received_msg_;
  Eigen::Vector2d goal_;
};

#endif