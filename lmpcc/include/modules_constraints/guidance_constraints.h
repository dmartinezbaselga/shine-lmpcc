/**
 * @file guidance_constraints.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Controller Module for computing guidance trajectories in the state-space and using them to construct
 * constraints (@see python_forces_code/modules.py)
 * @date 2022-09-23 (documented)
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __GUIDANCE_CONSTRAINTS_H__
#define __GUIDANCE_CONSTRAINTS_H__

#include <generated/submodules.h>
#include <modules_constraints/linearized_constraints.h>
#include <modules_objectives/reference_path.h>

#include <guidance_planner/global_guidance.h>

class RealTimeData;

/** @brief Save all the relevant results for a parallel solver in one place */
struct SolverResult
{
  int exit_code;
  double objective;
  bool success;

  int guidance_ID;

  void Reset()
  {
    success = false;
    objective = 1e10;
    exit_code = -1;

    guidance_ID = -1;
  }
};

/**
 * @brief Homotopy Guidance controller module extends from the reference path ControlModule to implement MPCC over the
 * trajectory but starting from the robot
 */
class GuidanceConstraints : public ReferencePath
{
public:
  GuidanceConstraints(ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle);
  ~GuidanceConstraints() = default;

public:
  /** @brief Update the module (any computations that need to happen before setting solver parameters) */
  void Update(SolverInterface *solver_interface, RealTimeData &data) override;

  void SetParameters(SolverInterface *solver, const RealTimeData &data, int N_iter, int &param_idx) override;

  /**
   * @brief Override to define a custom optimization loop. Note that there can only be ONE customized optimization.
   *
   * @return int exit_code of the solver, return any exit_code other than "EXIT_CODE_NOT_OPTIMIZED_YET" to define this
   * as a custom optimization
   */
  int Optimize(SolverInterface *solver_interface) override; // Default: no custom optimization

  /** @brief Visualize the computations in this module  */
  void Visualize() override;

  /** @brief Load obstacles into the Homotopy module */
  void OnDataReceived(SolverInterface *solver_interface, RealTimeData &data, std::string &&data_name) override;

  void OnReset(SolverInterface *solver_interface) override;

  void ReconfigureCallback(SolverInterface *solver_interface, lmpcc::PredictiveControllerConfig &config, uint32_t level, bool first_callback) override;

  void ExportData(RosTools::DataSaver &data_saver) override;

  void GetMethodName(std::string &name) override;

private: // Private functions
  struct LocalPlanner
  {
    int id;
    std::unique_ptr<LinearizedConstraints> guidance_constraints;   // Keep the solver in the topology
    std::unique_ptr<GUIDANCE_CONSTRAINTS_TYPE> safety_constraints; // Avoid collisions

    std::unique_ptr<SolverInterface> solver; // Distinct solver for each planner
    SolverResult result;

    bool is_original_planner = false;
    bool disabled = true;

    LocalPlanner(int _id, ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle,
                 bool _is_original_planner = false);
  };

  void SetGoalCosts();

  void SetParameters(LocalPlanner &planner, const RealTimeData &data, int N_iter, int &param_idx);

  void InitializeSolverWithGuidance(SolverInterface *solver);

  int FindBestPlanner();

  void VisualizeOptimizedPlan(LocalPlanner &planner);

  void VisualizeWarmstartPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line);
  void VisualizeGuidedPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line);
  void VisualizeGMPCCPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line);
  void VisualizeLMPCCPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line);

private: // Member variables
  std::vector<LocalPlanner> planners_;

  std::unique_ptr<RosTools::ROSMarkerPublisher> plan_markers_;

  GuidancePlanner::GlobalGuidance global_guidance_;

  RealTimeData empty_data_;

  int parameter_start_index_;
  RealTimeData *data_ptr_;

  int best_planner_index_ = -1;

  bool goal_received_;
  Eigen::Vector2d goal_;

  std::vector<RosTools::SignalPublisher> signal_publishers_; /* Publish the objective of each trajectory with jsk_rviz_plugins */
};

#endif // __GUIDANCE_CONSTRAINTS_H__