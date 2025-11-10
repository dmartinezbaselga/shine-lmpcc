/**
 * @file scenario_main.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Base class for scenario constraints. This class exists to manage the computations per disc.
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef SCENARIO_MAIN_H
#define SCENARIO_MAIN_H

#include <lmpcc/types.h>

#include <scenario/gaussian_sampler.h>
#include <scenario/safe_horizon.h>
#include <scenario/scenario_manager.h>
#include <scenario/smpcc.h>

#include <ros_tools/helpers.h>


class ScenarioConstraints : public ControllerModule
{

public:
  /**
   * @brief Construct a new Scenario Main object
   *
   * @param nh
   * @param solver_interface solver
   * @param config parameters
   * @param x_discs offset of the discs
   * @param use_scenarios this class is disabled if FALSE
   */
  ScenarioConstraints(ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle);

  // ScenarioConstraints(const ScenarioConstraints &other) = delete;

public:
  /** @todo: integrate */
  double velocity_jackal_ = 0.;

  // Status of the scenario computations
  ScenarioStatus status_;

  // Support and removed support
  SupportSubsample support_subsample_;
  SupportSubsample removed_support_;

  RealTimeData *data_ptr_;

  std::vector<double> cost_history_; // For tracking the cost values through SQP iterations

  /**
   * @brief Checks if the samples are prepared
   *
   * @param solver_interface
   * @param data
   * @return true
   * @return false
   */
  bool ReadyForControl(SolverInterface *solver_interface, const RealTimeData &data) override;

  /**
   * @brief Main update function, calls update for each of the discs
   *
   * @param dynamic_obstacles Dynamic obstacle data
   * @param halfspaces Static obstacle data
   */
  void Update(SolverInterface *solver_interface, RealTimeData &data) override; // const std::vector<DynamicObstacle> &dynamic_obstacles, const std::vector<lmpcc_msgs::halfspace_array> &halfspaces);

  /**
   * @brief Insert computed constraints into the solver
   *
   * @param k time index
   * @param param_idx parameter index, is increased internally
   */
  void SetParameters(SolverInterface *solver_interface, const RealTimeData &data, int k, int &param_idx) override;

  void OnDataReceived(SolverInterface *solver_interface, RealTimeData &data, std::string &&data_name);

  virtual int Optimize(SolverInterface *solver_interface) override;

  /**
   * @brief Update and publish visuals
   */
  void Visualize() override;

  /**
   * @brief Export runtime data
   *
   * @param data_saver the data_saver object to add the data to
   */
  void ExportData(RosTools::DataSaver &data_saver) override;

  void GetMethodName(std::string &name) override;

private:
  // Runtime parameters for exporting data
  int parameter_index_;
  int support_estimate_;
  int iterations_;
  int removed_scenarios_;

  bool first_run_;

  std::vector<std::thread> disc_threads_; // Discs are multithreaded
  bool failed_to_find_safe_solution_;

  GaussianSampler sampler_;

  // This contains the actual classes doing the computations (S-MPCC or SH-MPC)
  std::vector<std::unique_ptr<ScenarioManager>> scenario_manager_;

  /**
   * @brief Sequentially run SQP and check the support for SH-MPC
   *
   * @return int exit_code
   */
  int SequentialScenarioIterations(SolverInterface *solver_interface);

  /**
   * @brief Sequentially run SQP without any checks (S-MPCC)
   *
   * @return int exit_code
   */
  int SimpleSequentialScenarioIterations(SolverInterface *solver_interface);

  /**
   * @brief Check if the status of a scenario class is the same for all discs
   *
   * @param expected_status The status to compare with
   * @param status_out The actual status
   * @return true If the status matched the expected one
   * @return false otherwise
   */
  bool IsScenarioStatus(ScenarioStatus &&expected_status, ScenarioStatus &status_out);

  /**
   * @brief Remove scenarios from the optimization
   *
   * @param support the current support
   * @param removed_scenarios the scenarios to be removed (removed scenarios are added to this)
   * @param num_scenarios_to_remove maximum number of scenarios to remove
   */
  void RemoveScenarios(SolverInterface *solver_interface, const SupportSubsample &support, SupportSubsample &removed_scenarios, int num_scenarios_to_remove);
};

#endif