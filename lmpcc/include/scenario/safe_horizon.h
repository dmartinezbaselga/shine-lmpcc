/**
 * @file trajectory_disc.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Code for disc-wise computations of Safe Horizon MPC (SH-MPC)
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef TRAJECTORY_DISC_H
#define TRAJECTORY_DISC_H

#include <geometry_msgs/PolygonStamped.h>

#include "scenario/polygon_search.h"
#include "scenario/safety_certifier.h"
#include "scenario/scenario_manager.h"

#include "ros_tools/data_saver.h"
#include "sampler.h"

class SafeHorizon : public ScenarioManager
{

public:
  /**
   * @brief Construct a new Trajectory Disc object
   *
   * @param nh
   * @param config parameters
   * @param disc_offset offset of this disc
   * @param enable_visualization visualization for this disc is enabled if TRUE
   */
  SafeHorizon(ros::NodeHandle &nh, predictive_configuration *config, const int disc_id, GaussianSampler &sampler, bool enable_visualization);

  // No copying
  SafeHorizon(const SafeHorizon &other) = delete;

public:
  /**
   * @brief Update the scenario constraints of this disc using Safe Horizon MPC
   *
   * @param solver_interface The solver class
   * @param dynamic_obstacles The obstacle positions and predictions
   * @param halfspaces Halfspaces to add to the constraints
   */
  void Update(SolverInterface *solver_interface, RealTimeData &data) override;

  /**
   * @brief Insert the computed constraints into the solver
   *
   * @param solver_interface The solver class
   * @param k_solver The stage to insert to (1 - N)
   * @param param_idx The parameter index
   */
  void SetParameters(SolverInterface *solver_interface, const RealTimeData &data, int k_solver, int &param_idx) override;

  /**
   * @brief Compute the active constraints and add them to the aggregated list
   *
   * @param solver_interface The solver class
   * @param active_constraints_aggregate Current support aggregate
   * @return true If feasible
   * @return false If infeasible
   */
  bool computeActiveConstraints(SolverInterface *solver_interface, SupportSubsample &active_constraints_aggregate) override;

  /**
   * @brief Remove active constraints from the optimization
   *
   * @param active_constraints The active constraints as a support subsample object
   * @param num_scenarios_to_remove The number of scenarios to remove
   * @return SupportSubsample& The removed scenarios
   */
  SupportSubsample &RemoveActiveConstraints(const SupportSubsample &active_constraints, int num_scenarios_to_remove) override;

  /**
   * @brief Check if the optimized solution is feasible w.r.t. to the scenarios
   *
   * @param solver_interface The solver class
   * @return true If feasible
   * @return false If not feasible
   */
  bool isFeasible(SolverInterface *solver_interface);

  /**
   * @brief Insert a backup plan into the solver
   *
   * @param solver_interface The solver class
   * @param vel Velocity of the robot
   */
  void useBackupPlan(SolverInterface *solver_interface, double velocity, double deceleration) override;

  std::vector<double> &GetAreas() override { return polygon_areas_; };

  /**
   * @brief Visualize the processes in this class
   */
  void Visualize() override;

private:
  // Parameters that will be set via config
  u_int S; // sampling count
  u_int N; // Prediction horizon

  // Received from constructor
  predictive_configuration *config_;
  bool enable_visualization_; // Visualization is enabled if TRUE

  // The visualisation class
  std::unique_ptr<RosTools::ROSMarkerPublisher> ros_markers_;

  // Collision discs
  // std::unique_ptr<VehicleDisc> disc_;
  int disc_id_;
  std::vector<VehicleRegion> plan_;
  std::vector<DynamicObstacle> *obstacles_;

  bool is_feasible_;

  // Threads for multithreading per stage (disabled)
  std::vector<std::thread> scenario_threads_;

  // A pointer to the scenarios
  std::vector<trajectory_sample> *scenarios_;

  // Intermediate distance computation (reuse for obstacles, but threaded for k)
  std::vector<Eigen::ArrayXd> diffs_x_;
  std::vector<Eigen::ArrayXd> diffs_y_;
  std::vector<Eigen::ArrayXd> distances_;

  // Constraint vectors Ax <= b
  std::vector<Eigen::ArrayXd> a1_;
  std::vector<Eigen::ArrayXd> a2_;
  std::vector<Eigen::ArrayXd> b_;

  // The x, y value of all constraints at a left and right point from the vehicle (for the polygon search)
  std::vector<Eigen::ArrayXd> y_left_, y_right_;
  std::vector<double> x_left_, x_right_;

  // Classes for computing the minimal polygon
  std::vector<PolygonSearch> polytopes_;

  // A vector with the sample and obstacle indices per scenario
  std::vector<std::vector<Scenario>> scenario_indices_;

  // Meta-data of constructed constraints
  std::vector<std::vector<ScenarioConstraint>> constraints_;

  // Joint radii w.r.t. each obstacle (including vehicle radius)
  std::vector<double> radii_;

  // Vehicle poses used in verification
  std::vector<Eigen::Vector2d> verify_poses_;

  // Old intersects used in removal
  std::vector<std::vector<Eigen::Vector2d *>> old_intersects_;

  // Tracking of infeasible scenarios
  std::vector<bool> distances_feasible_;
  std::vector<std::vector<Eigen::Vector2d>> infeasible_scenario_poses_;
  std::vector<std::vector<int>> infeasible_scenario_idxs_;

  std::vector<double> polygon_areas_;

  /**
   * @brief Clear data from previous computations
   */
  void clearAll();

  /**
   * @brief Load external data, retrieve scenarios and prepare for the update
   *
   * @param solver_interface
   * @param dynamic_obstacles
   * @param halfspaces
   */
  void LoadData(SolverInterface *solver_interface, RealTimeData &data);

  // COMPUTATIONS FOR EACH K, OBSTACLE
  /**
   * @brief Compute distances to all scenarios
   *
   * @param k time index
   * @param obstacle_id obstacle index
   */
  void computeDistances(int k, int obstacle_id);

  /**
   * @brief Check feasibility based on computed distances
   *
   * @param k time index
   * @param obstacle_id obstacle index
   */
  void checkFeasibilityByDistance(int k, int obstacle_id);

  /**
   * @brief Use distances, diff_x_ and diff_y_ to compute constraints for all scenarios
   *
   * @param k time index
   * @param obstacle_id obstacle index
   */
  void computeHalfspaces(int k, int obstacle_id);

  /**
   * @brief Construct the polytope given all computed constraints
   *
   * @param k time index
   * @param obstacle_id obstacle index
   */
  void constructPolytopes(int k, const std::vector<RosTools::Halfspace> &halfspaces);

  // PROJECTION SCHEMES
  /**
   * @brief Push the initial plan away from scenarios if infeasible (orthogonal to the vehicle plan)
   *
   * @param solver_interface solver where the plan is stored and updated
   */
  void PushAlgorithm(SolverInterface *solver_interface);

  /**
   * @brief Project the plan to feasibility w.r.t. to the scenario constraints using (Cyclic) Douglas-Rachford Splitting
   *
   * @param solver_interface
   * @param halfspaces Static obstacles, to ensure also feasibility w.r.t. these obstacles
   */
  void DRProjection(SolverInterface *solver_interface, const std::vector<std::vector<RosTools::Halfspace>> &halfspaces);

  // Convert samples to vectors for visuals
  Eigen::Vector3d getScenarioLocation(const int &k, const int &obstacle_index, const int &scenario_index);
  Eigen::Vector2d getScenarioLocation2D(const int &k, const int &obstacle_index, const int &scenario_index);

  // Visualisation methods
  void visualiseEllipsoidConstraints(){};

  void visualizeAllConstraints();    // Visualize all scenario constraints
  void visualiseScenarios();         // Visualize all scenarios (very slow)
  void visualiseSelectedScenarios(); // Visualize support scenarios
  void visualisePolygonScenarios();  // Visualize all scenarios that contribute to the final constraints (slow)
  void visualisePolygons();          // Visualize the final constraints

  // Call to publish all scenario visuals
  void publishVisuals();
};

#endif