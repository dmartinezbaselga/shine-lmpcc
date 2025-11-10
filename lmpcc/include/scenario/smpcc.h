/**
 * @file stage_scenario.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Implements S-MPCC (https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9410362)
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef SMPCC_H
#define SMPCC_H

#include "scenario/scenario_manager.h"

#include "ros_tools/helpers.h"
#include "local_polygon/local_polygon.h"
#include "scenario/safety_certifier.h"
#include "scenario/sampler.h"

#include <Eigen/Cholesky>
#include <Eigen/Eigen>

/** Set to 0 to compile faster without debug information */
#define SCENARIO_DEBUG 0
#if SCENARIO_DEBUG == 1
#define SCENARIO_INFO(msg)                \
  if (config_->debug_output_)             \
  {                                       \
    ROS_INFO_STREAM("[S-MPCC]: " << msg); \
  }
#define SCENARIO_INFO_STREAM(msg)         \
  if (config_->debug_output_)             \
  {                                       \
    ROS_INFO_STREAM("[S-MPCC]: " << msg); \
  }
#else
#define SCENARIO_INFO(msg)
#define SCENARIO_INFO_STREAM(msg)
#endif

class SMPCC : public ScenarioManager
{

public:
  SMPCC(ros::NodeHandle &nh, predictive_configuration *config, const int disc_id, GaussianSampler &sampler, bool enable_visualization);

private:
  // Parameters that will be set via config
  u_int S; // sampling count
  u_int N; // Prediction horizon

  int R_; // Scenario removal
  int l_;

  // Received from constructor
  predictive_configuration *config_;
  bool enable_visualization_;

  // The visualisation class
  std::unique_ptr<RosTools::ROSMarkerPublisher> ros_markers_;

  int disc_id_;
  std::vector<VehicleRegion> plan_;

  // New
  // A pointer to the scenarios
  std::vector<trajectory_sample> *scenarios_;

  // Intermediate distance computation (reuse for obstacles, but threaded for k)
  // std::vector<Eigen::ArrayXd> diffs_x_;
  // std::vector<Eigen::ArrayXd> diffs_y_;
  // std::vector<Eigen::ArrayXd> distances_;

  bool is_feasible_;

  std::vector<int> sort_indices_;

  // // These are for all obstacles as well
  // std::vector<Eigen::ArrayXd> a1_;
  // std::vector<Eigen::ArrayXd> a2_;
  // std::vector<Eigen::ArrayXd> b_;

  std::vector<LocalPolygon::PolygonConstructor> local_polygons_;

  // A vector with the sample and obstacle indices per scenario
  // std::vector<std::vector<Scenario>> scenario_info_;

  // Constraints constructed from the scenarios
  // std::vector<std::vector<ScenarioConstraint>> constraints_;

  // The x, y value of all constraints at a left and right point from the vehicle (for the polygon search)
  // std::vector<Eigen::ArrayXd> y_left_;
  // std::vector<Eigen::ArrayXd> y_right_;
  // std::vector<double> x_left_, x_right_;

  // Obstacle msgs
  std::vector<DynamicObstacle> *obstacles_;
  std::vector<double> radii_;

  // Threads for multithreading per stage
  std::vector<std::thread> scenario_threads_;

  // Active obstacles
  std::vector<std::vector<int>> active_obstacle_indices_;
  std::vector<int> active_obstacle_count_;

  // Vehicle poses when project outside of obstacles
  std::vector<Eigen::Vector2d> projected_poses_;

  // Areas of the regions
  std::vector<double> areas_;

  /**
   * @brief Convert scenarios to constraints
   *
   * @param k time step k
   * @param halfspaces external static half-spaces
   */
  void scenariosToConstraints(int k, const std::vector<RosTools::Halfspace> &halfspaces);

  /**
   * @brief Remove scenarios based on distance
   *
   * @param k time step k
   */
  void removeScenariosBySorting(int k);

  /**
   * @brief Check feasibility of the scenarios
   *
   * @param k time step k
   */
  void feasibilityCheck(int k);

  /**
   * @brief Projection to obtain a feasible initial plan
   *
   * @param solver_interface solver interface
   */
  void PushAlgorithm(SolverInterface *solver_interface, const RealTimeData &data);

  /**
   * @brief Getter to get a scenario position
   *
   * @param k time step k
   * @param obstacle_index the obstacle index
   * @param scenario_index the scenario index
   * @return Eigen::Vector3d output position vector in 3D
   */
  Eigen::Vector3d getScenarioLocation(const int &k, const int &obstacle_index, const int &scenario_index);
  Eigen::Vector3d getScenarioLocation(const int &k, const Scenario &scenario);
  int constraintIndexToScenarioIndex(int constraint_index);

  // Visualisation methods
  void visualiseConstraints();
  void visualiseScenarios();
  void visualiseRemovedScenarios(const std::vector<int> &indices_to_draw);
  void visualiseSelectedScenarios(const std::vector<int> &indices_to_draw);
  void visualisePolygons();
  void visualisePolygonScenarios();
  void visualiseProjectedPosition();

  // Call to publish all scenario visuals
  void publishVisuals();

public:
  /**
   * @brief Updates inequality constraints defined by S-MPCC
   *
   * @param solver_interface the solver interface
   * @param dynamic_obstacles dynamic obstacle positions and predictions
   * @param halfspaces static obstacles as linear constraints
   */
  void Update(SolverInterface *solver_interface, RealTimeData &data) override;

  /**
   * @brief Visualize the constraints
   */
  void Visualize();

  /**
   * @brief Insert constraints into the solver
   *
   * @param solver_interface the solver interface
   * @param k time step k
   * @param param_idx parameter index (increased internally)
   */
  void SetParameters(SolverInterface *solver_interface, const RealTimeData &data, int k, int &param_idx) override;

  /**
   * @brief Use a backup plan when infeasible
   *
   * @param solver_interface the solver interface
   * @param vel the velocity
   * @param deceleration the deceleration
   */
  void useBackupPlan(SolverInterface *solver_interface, double vel, double deceleration) override;

  // Get the area of a particular polytope
  std::vector<double> &GetAreas() override { return areas_; };
};

#endif