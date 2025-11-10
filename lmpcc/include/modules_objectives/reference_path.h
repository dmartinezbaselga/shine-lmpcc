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

#ifndef REFERENCE_PATH_H
#define REFERENCE_PATH_H

#include <ros_tools/helpers.h>
#include <lmpcc/types.h>

// #include <lmpcc/lmpcc_configuration.h>
// #include <lmpcc/PredictiveControllerConfig.h>

#include <ros_tools/ros_visuals.h>

#include <lmpcc_msgs/halfspace_array.h>
#include <lmpcc_msgs/halfspace.h>

#include <nav_msgs/Path.h>
#include <Eigen/Dense>

// splines
#include <tkspline/spline.h>
#include <lmpcc/Clothoid.h>

class predictive_configuration;
class SolverInterface;

// Whens earching for the closest point on the path, this variable indicates the distance that the algorithm searches
// behind the current spline point.
#define MAX_STEP_BACK_TOLERANCE 0.1f

class ReferencePath : public ControllerModule
{
public:
  /**
   * @brief Initialize the reference path, reading the path from file.
   *
   * @param nh nodehandle
   * @param config parameters
   */
  ReferencePath(ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle);

public:
  // Current spline index
  unsigned int spline_index_;
  unsigned int waypoints_size_;

  double minimal_s_ = 0;

  bool goal_reached_ = false;
  bool first_run_ = true;

  int segments = 0;

  // Waypoints x, y, psi
  std::vector<double> x_, y_, psi_;

  // Output splines
  tk::spline ref_path_x_, ref_path_y_;
  std::unique_ptr<RosTools::CubicSpline2D<tk::spline>> reference_path_; // Combines both splines and provides some useful wrapper
                                                                        // functions

  // Spline s, x, y
  std::vector<double> ss_, xx_, yy_; // ss is the distance at which each spline begins

  double dist_spline_pts_;

  nav_msgs::Path spline_msg_;

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

  /**
   * @brief Construct a reference path from the stored values of x_, y_, psi_
   */
  void InitPath();
  /**
   * @brief Construct a reference path with clothoid from the given values of x_, y_, psi_
   *
   * @param x list of x
   * @param y list of y
   * @param psi list of orientations
   */
  void InitPath(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &psi);

  /**
   * @brief Construct a reference path without clothoid from the given values of x_, y_
   *
   * @param x list of x
   * @param y list of y
   */
  void InitPath(const std::vector<double> &x, const std::vector<double> &y);

  /**
   * @brief Public wrapper for <RecursiveClosestPointSearch>"(SolverInterface *solver_interface_ptr, unsigned int
   * cur_traj_i, double &s_guess, double window, int n_tries)" Also visualizes the spline index found.
   *
   * @param solver_interface_ptr solver interface
   * @param s_guess guessed spline value
   * @param window window to search in w.r.t. the guess
   * @param n_tries number of recursions allowed
   */
  void UpdateClosestPoint(SolverInterface *solver_interface_ptr, double &s_guess, double window, int n_tries);

  /**
   * @brief Find the closest point over the entire reference path
   *
   * @param solver_interface_ptr solver interface to retrieve the vehicle position from
   */
  void InitializeClosestPoint(SolverInterface *solver_interface_ptr);

  /**
   * @brief Is the current spline segment at an end?
   *
   * @param index Current spline index
   * @return true If end of current spline segment was reached
   */
  bool EndOfCurrentSpline(double index);

  /**
   * @brief Checks if the end of the path was reached
   *
   * @return true If end of the path was reached
   */
  bool ReachedEnd();

  /**
   * @brief Construct linear road constraints by linearizing the spline boundaries w.r.t. the vehicle position
   *
   * @param solver_interface solver interface to retrieve the vehicle positions from
   * @param halfspaces_out Output halfspaces as vector with entries for each time step
   */
  void ConstructRoadConstraints(SolverInterface *solver_interface, std::vector<std::vector<RosTools::Halfspace>> &halfspaces_out);

  // Visualization
  /**
   * @brief Draw the reference path (line) and its orientations (arrows)
   */
  void PublishReferencePath();

  /**
   * @brief Publish cubes marking the current spline index followed by the vehicle
   */
  void PublishCurrentSplineIndex();

  /**
   * @brief Visualize the linearized road boundary constraints (the ones gives) - Debug functionality only
   *
   * @param solver_interface_ptr solver
   * @param halfspaces_out the constraints to plot
   */
  void PublishLinearRoadBoundaries(SolverInterface *solver_interface_ptr,
                                   const std::vector<RosTools::Halfspace> &halfspaces_out);

  /**
   * @brief Visualize road boundaries as a spline (i.e., not linearized)
   *
   * @param solver_interface_ptr solver interface
   */
  void VisualizeRoad();

private:
  /* Visuals */
  std::unique_ptr<RosTools::ROSMarkerPublisher> ros_markers_reference_path_, ros_markers_reference_arrows_;
  std::unique_ptr<RosTools::ROSMarkerPublisher> ros_markers_splineindex;
  std::unique_ptr<RosTools::ROSMarkerPublisher> ros_markers_linearboundaries;
  std::unique_ptr<RosTools::ROSMarkerPublisher> ros_markers_road_limits;

  // Search window parameters for the spline
  double window_size_ = 2;
  int n_search_points_ = 20;

  /**
   * @brief Read a reference path from a configuration file
   */
  void ReadReferencePath();

  /**
   * @brief Construct a reference path from a set of waypoints (x, y, psi)
   *
   * @param x list of x values
   * @param y list of y values
   * @param psi list of orientation values
   */
  void ConstructReferencePath(const std::vector<double> &x, const std::vector<double> &y,
                              const std::vector<double> &psi); // Selects which method to use

  /**
   * @brief Called from <ReadReferencePath>"()". First fits a clothoid to smoothen the waypoints, then fits a cubic
   * spline.
   *
   * @param x list of x values
   * @param y list of y values
   * @param psi list of orientation values
   */
  void ConstructReferencePathWithClothoid(const std::vector<double> &x, const std::vector<double> &y,
                                          const std::vector<double> &psi);

  /**
   * @brief Called from <ReadReferencePath>"()". Fits a cubic spline (no clothoid)
   *
   * @param x list of x values
   * @param y list of y values
   * @param psi list of orientation values
   */
  void ConstructReferencePathWithoutClothoid(const std::vector<double> &x, const std::vector<double> &y);

  /**
   * @brief Search for the closest point to the vehicle
   *
   * @param solver_interface_ptr Solver interface to retrieve the vehicle position
   * @param cur_traj_i Current spline index
   * @param s_guess Guessed spline value
   * @param window Window to search in w.r.t. the guess
   * @param n_tries Number of recursions allowed
   * @return int The closest spline index
   */
  int RecursiveClosestPointSearch(SolverInterface *solver_interface_ptr, unsigned int cur_traj_i, double &s_guess,
                                  double window, int n_tries, int num_recursions);

  double FindClosestSRecursively(const Eigen::Vector2d &pose, double low, double high, int num_recursions);
};

#endif