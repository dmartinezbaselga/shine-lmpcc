/**
 * @file scenario_manager.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Base class for S-MPCC and SH-MPC (mostly virtual)
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef SCENARIO_MANAGER_H
#define SCENARIO_MANAGER_H

#include "lmpcc/dynamic_obstacle.h"
#include "lmpcc_configuration.h"
#include "ros_tools/helpers.h"
#include "ros_tools/ros_visuals.h"
#include <lmpcc/types.h>

#include "scenario/gaussian_sampler.h"

#include "lmpcc_solver/collision_region.h"

#include <lmpcc_msgs/halfspace.h>
#include <lmpcc_msgs/halfspace_array.h>
#include <lmpcc_msgs/lmpcc_obstacle.h>
#include <lmpcc_msgs/lmpcc_obstacle_array.h>

enum class ScenarioStatus
{
  SUCCESS = 0,
  PROJECTED_SUCCESS,
  BACKUP_PLAN,
  INFEASIBLE,
  DATA_MISSING,
  RESET
};

class ScenarioManager
{

public:
protected:
  Eigen::Vector2d disc_offset_;

  // Vehicle poses when project outside of obstacles
  std::vector<Eigen::Vector2d> poses_;
  std::vector<double> orientations_;

  // void retrievePoses(SolverInterface *solver_interface, bool shift, double disc_offset)
  // {
  //     // X2-XN
  //     for (size_t k = 0; k < solver_interface->FORCES_N; k++)
  //     {
  //         // Since we shift the trajectory, we need to use the last stage twice
  //         int k_capped;
  //         if (shift)
  //             k_capped = k == solver_interface->FORCES_N - 1 ? k : k + 1;
  //         else
  //             k_capped = k;

  //         // Get the orientation and pose at stage k
  //         orientations_[k] = solver_interface->psi(k_capped);
  //         poses_[k] = Eigen::Vector2d(
  //             solver_interface->x(k_capped) + disc_offset * std::cos(solver_interface->psi(k_capped)),
  //             solver_interface->y(k_capped) + disc_offset * std::sin(solver_interface->psi(k_capped)));
  //     }
  // };

  // void retrievePoses(SolverInterface *solver_interface, bool shift)
  // {
  //     retrievePoses(solver_interface, shift, disc_offset_(0));
  // };

  // void retrievePoses(SolverInterface *solver_interface, bool shift, const Disc &disc)
  // {
  //     retrievePoses(solver_interface, shift, disc.offset_);
  // };

public:
  // To unify for trajectory / marginal, can add k to Scenario structure
  SupportSubsample support_subsample_;
  SupportSubsample removed_scenarios_;
  GaussianSampler *sampler_;
  std::vector<DynamicObstacle> copied_obstacles_; // Thread safe obstacles

  ScenarioStatus status_;
  bool polygon_failed_;

  double velocity_; // Should be removed later (just replace all the specific data with a system interface pointer)

  // Todo: merge
  virtual void Update(SolverInterface *solver_interface, RealTimeData &data) = 0;

  virtual void SetParameters(SolverInterface *solver_interface, const RealTimeData &data, int k, int &param_idx) = 0;

  virtual void Visualize() = 0;

  virtual void resolveInfeasibility(SolverInterface *solver_interface){};

  virtual bool computeActiveConstraints(SolverInterface *solver_interface, SupportSubsample &active_constraints_aggregate) { return false; };

  virtual void ProjectToFeasible(SolverInterface *solver_interface){};

  virtual bool isFeasible(SolverInterface *solver_interface) { return false; };

  virtual SupportSubsample &RemoveActiveConstraints(const SupportSubsample &active_constraints, int num_scenarios_to_remove) { return removed_scenarios_; };

  virtual void ExportDebugProblem(SolverInterface *solver_interface, int k){};

  virtual void useBackupPlan(SolverInterface *solver_interface, double velocity, double deceleration){};

  virtual std::vector<double> &GetAreas() = 0;
};

#endif