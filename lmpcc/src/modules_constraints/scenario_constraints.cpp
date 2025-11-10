#include "modules_constraints/scenario_constraints.h"

#include <lmpcc/lmpcc_configuration.h>
#include <lmpcc_solver/SolverInclude.h>

ScenarioConstraints::ScenarioConstraints(ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle) //, const Eigen::VectorXd &x_discs, bool use_scenarios)
    : ControllerModule(nh, config, vehicle)
{
  type_ = ModuleType::CONSTRAINT;

  status_ = ScenarioStatus::RESET;

  // Initialize the safety certifier
  SafetyCertifier::Get().init(config_);

  // Initialize the scenario handlers for the discs
  scenario_manager_.resize(vehicle_->discs_.size());

  // Note: order of sampler and scenario initialization is different!
  if (config_->use_trajectory_sampling_) // SH-MPC
  {
    // Initialize the classes for each disc
    for (size_t i = 0; i < vehicle_->discs_.size(); i++)
      scenario_manager_[i].reset(new SafeHorizon(nh, config_, i, sampler_, config_->discs_to_draw_[i]));

    // Initialize samples
    Sampler::Get().Init(nh, config_);
    sampler_.Init(config_);
  }
  else
  {
    Sampler::Get().Init(nh, config_);
    sampler_.Init(config_);

    // Initialize the classes for each disc
    for (size_t i = 0; i < vehicle_->discs_.size(); i++)
      scenario_manager_[i].reset(new SMPCC(nh, config_, i, sampler_, config_->discs_to_draw_[i]));
  }

  // Make sure parameters are not used before they are set
  parameter_index_ = -1;
  support_estimate_ = -1;
  removed_scenarios_ = -1;
  iterations_ = 0;

  first_run_ = true;
}

bool ScenarioConstraints::ReadyForControl(SolverInterface *solver_interface, const RealTimeData &data)
{
  if ((config_->use_trajectory_sampling_ && (!sampler_.SamplesReady())) || ((!config_->use_trajectory_sampling_) && data.dynamic_obstacles_.size() < (unsigned int)config_->max_obstacles_))
  {
    LMPCC_WARN("There are no Sampled Scenarios Yet!");
    status_ = ScenarioStatus::DATA_MISSING;
    return false;
  }

  return true;
}

void ScenarioConstraints::Update(SolverInterface *solver_interface, RealTimeData &data)
{
  LMPCC_INFO("ScenarioConstraints::Update()");

  cost_history_.clear();

  data_ptr_ = &data;

  // Update the scenarios
  /*disc_threads_.clear();
  for (auto &disc : scenario_manager_)
  {
      disc->velocity_ = solver_interface->State().v(); // velocity_jackal_;
      // if(config_->use_trajectory_sampling_)

      if (config_->multithread_scenarios_)
          disc_threads_.emplace_back(&SafeHorizon::Update, (SafeHorizon *)(disc.get()), solver_interface, std::ref(data));
      else
          disc->Update(solver_interface, data);
      // else
      //     disc_threads_.emplace_back(&StageScenario::Update, (StageScenario*)disc.get(), solver_interface, data);
  }

  if (config_->multithread_scenarios_)
  {
      for (size_t i = 0; i < disc_threads_.size(); i++)
      {
          disc_threads_[i].join();
          // std::cout << "Joined " << i << std::endl;
      }
  }*/

  // #pragma omp parallel for -> Does not stack with the parallel loops internally!
  for (auto &disc : scenario_manager_)
  {
    disc->Update(solver_interface, data);
  }

  // Get the areas of the polygons of the front disc
  auto &disc = scenario_manager_[0];
  auto &areas = disc->GetAreas();

  double max_width = config_->two_way_road_ ? config_->road_width_left_ * 3 + config_->road_width_right_ : config_->road_width_left_ + config_->road_width_right_;
  max_width -= 2 * config_->vehicle_width_ / 2.; // Compensate for the space that the vehicle needs
  double max_area = max_width * (2 * config_->polygon_range_);

  // Scale the velocity reference by the size of the regions
  data.reference_velocity_.resize(areas.size());
  for (size_t k = 0; k < areas.size(); k++)
  {
    if (config_->scale_reference_velocity_)
      data.reference_velocity_[k] = config_->reference_velocity_ * std::min(areas[k] / max_area, 1.0);
    else
      data.reference_velocity_[k] = config_->reference_velocity_;
  }

  // Checks if there were any problems in constructing the polytopes
  if (IsScenarioStatus(ScenarioStatus::SUCCESS, status_))
    return;

  LMPCC_WARN_ALWAYS("Scenario Main: Not all discs are feasible");
}

// How to allow for two statuses.
bool ScenarioConstraints::IsScenarioStatus(ScenarioStatus &&expected_status, ScenarioStatus &status_out)
{
  status_out = expected_status;

  for (auto &disc : scenario_manager_)
  {
    if (disc->status_ != expected_status)
    {
      status_out = disc->status_;
      return false;
    }
  }

  return true;
}

void ScenarioConstraints::OnDataReceived(SolverInterface *solver_interface, RealTimeData &data, std::string &&data_name)
{
  if (data_name == "Dynamic Obstacles")
  {
    if (config_->use_trajectory_sampling_)
    {
      sampler_.IntegrateAndTranslateToMeanAndVariance(data, solver_interface->DT);
    }
    else if (config_->use_real_samples_)
    {
      Sampler::Get().SampleJsonRealTrajectories(data); // Sample scenarios from the associated partitions
    }
  }
}

void ScenarioConstraints::GetMethodName(std::string &name)
{
  if (config_->use_trajectory_sampling_)
    name = "SH_MPC";
  else
    name = "SMPCC";
}

void ScenarioConstraints::SetParameters(SolverInterface *solver_interface, const RealTimeData &data, int k, int &param_idx)
{
  parameter_index_ = param_idx;

  // Insert scenario constraints
  for (auto &disc : scenario_manager_)
    disc->SetParameters(solver_interface, data, k, param_idx);
}

int ScenarioConstraints::Optimize(SolverInterface *solver_interface)
{
  LMPCC_INFO("Running ScenarioConstraint Optimization");
  if (solver_interface->use_sqp_solver)
  {
    if (config_->use_trajectory_sampling_)
      return SequentialScenarioIterations(solver_interface); // Safe Horizon MPC
    else
      return SimpleSequentialScenarioIterations(solver_interface); // S-MPCC (SQP)
  }
  else
  {
    if (config_->use_trajectory_sampling_)
      ROS_ERROR_ONCE("SH-MPC: Warning: SH-MPC needs to use SQP to guarantee safety!");

    return ControllerModule::Optimize(solver_interface); // Run the regular optimization (Baseclass!)
  }
}

int ScenarioConstraints::SequentialScenarioIterations(SolverInterface *solver_interface)
{
  int exit_code = -1; // Returned exit code

  int prev_support_estimate = 0;
  bool feasible = true;
  failed_to_find_safe_solution_ = false;
  int new_support = 0;

  support_estimate_ = 0;
  removed_scenarios_ = 0;

  SupportSubsample support_estimate(SafetyCertifier::Get().GetMaxSupport()); // Aggregates the support over all discs and iterations
  SupportSubsample removed_scenarios(config_->removal_count_);

  // std::cout << "------------------------------\n";
  // Every iteration has variables (x*, cost, support, risk)
  for (iterations_ = 0; iterations_ < config_->max_iterations_; iterations_++)
  {
    if (config_->debug_output_ && (iterations_ == 0 || new_support > 0)) // Print an update at the start and when the last iteration had support
      support_estimate.PrintUpdate(SafetyCertifier::Get().GetSafeSupportBound(), removed_scenarios, config_->removal_count_, iterations_);

    // We load the previous trajectory in the first iterations and reuse the previous iteration in later iterations
    if (iterations_ == 0)
    {
      solver_interface->setReinitialize(true); // Initialize with the shifted trajectory
    }
    else if (iterations_ >= 1)
    {
      solver_interface->setReinitialize(false); // Use the current SQP iterate x*
    }

    // Solve an iteration of the SQP problem and load the output
    {
      PROFILE_SCOPE("Solver Iteration");
      exit_code = solver_interface->solve(1, config_->terminate_eq_tol_);

      solver_interface->printSolveInfo();
      cost_history_.push_back(solver_interface->Cost());
    }

    // solver_interface_->printSolveInfo();

    // Detect what constraints are of support (aggregated over all discs)
    feasible = true;
    for (auto &disc : scenario_manager_)
      feasible = feasible & disc->computeActiveConstraints(solver_interface, support_estimate);

    // If the solution was not feasible, stop (it cannot recover)
    if ((!feasible) || (exit_code != 1))
    {
      solver_interface->printSolveInfo();

      ROS_ERROR_STREAM("Scenario Main: SQP iterate became infeasible (exit_code = " << exit_code << ")");
      removed_scenarios_ = removed_scenarios.support_subsample_size_;
      break;
    }

    // Insert the support constraints to the discs for visualization (Note the last disc already has this information)
    for (auto &disc : scenario_manager_)
      disc->support_subsample_.MergeWith(support_estimate);

    // Update the support parameters
    support_estimate_ = support_estimate.support_subsample_size_;

    new_support = support_estimate_ - prev_support_estimate;

    // If the support bound is exceeded in the current iteration -> Terminate the optimization
    if (config_->enable_termination_ && support_estimate_ > SafetyCertifier::Get().GetSafeSupportBound())
    {

      // Return the previous solution
      if (iterations_ > 0)
      {
        // LMPCC_INFO_STREAM
        LMPCC_WARN("Scenario Main: Terminated (Support: " << support_estimate_ << " > Bound: " << SafetyCertifier::Get().GetSafeSupportBound() << " In iteration: " << iterations_ + 1 << " / "
                                                          << config_->max_iterations_ << ")");

        if (solver_interface->HasSafeSolution())
        {

          solver_interface->UseLastSolution();

          // Remove last data for other important variables
          cost_history_.pop_back();
          iterations_--;                             // Last iteration does not count
          support_estimate_ = prev_support_estimate; // Reset support count

          failed_to_find_safe_solution_ = false;
        }
        else
        {
          failed_to_find_safe_solution_ = true;
          // LMPCC_ERROR_STREAM("THERE WAS NO SAFE PREVIOUS SOLUTION (Failed to converge fast enough before terminating)");
        }
      }
      else
      {
        failed_to_find_safe_solution_ = true;
        // LMPCC_ERROR_STREAM("THERE WAS NO SAFE PREVIOUS SOLUTION (terminated after 0 iterations)");
      }

      // solver_interface->printSolveInfo();

      break;
    }

    prev_support_estimate = support_estimate_;

    // Only remove scenarios if convergence is sufficient
    bool more_or_less_converged = false;
    if (iterations_ == 0 || (cost_history_.back() / cost_history_[cost_history_.size() - 2] * 100.0 > 80.0))
      more_or_less_converged = true;

    // We can remove scenarios if
    if (config_->enable_scenario_removal_                                      /* Removal is enabled */
        && more_or_less_converged && new_support > 0                           /* And, There are new scenarios */
        && removed_scenarios.support_subsample_size_ < config_->removal_count_ /* And, we will not violate the specified removal bound */
        && iterations_ != config_->max_iterations_ - 1)                        /* And if there are still iterations after this */
    {
      int num_scenarios_to_remove = std::min(config_->removal_count_ - removed_scenarios.support_subsample_size_, new_support);

      RemoveScenarios(solver_interface, support_estimate, removed_scenarios, num_scenarios_to_remove);
    }
  }

  removed_scenarios_ = removed_scenarios.support_subsample_size_;

  return exit_code;
}

void ScenarioConstraints::RemoveScenarios(SolverInterface *solver_interface, const SupportSubsample &support, SupportSubsample &removed_scenarios, int num_scenarios_to_remove)
{

  PROFILE_FUNCTION();
  LMPCC_INFO("Scenario Main: Removing Scenarios");

  // Remove the active scenarios from the set and reconstruct the polytope
  for (auto &disc : scenario_manager_)
  {
    removed_scenarios = disc->RemoveActiveConstraints(support, num_scenarios_to_remove);
  }

  // Insert the updated polytopes
  for (size_t k = 0; k < solver_interface->FORCES_N; k++)
  {
    int param_idx = parameter_index_;
    SetParameters(solver_interface, *data_ptr_, k + 1, param_idx); // k+1 to skip the initial state
  }
}

void ScenarioConstraints::ExportData(RosTools::DataSaver &data_saver)
{
  if (first_run_)
  {
    data_saver.AddData("sample_size", SafetyCertifier::Get().GetSampleSize());
    data_saver.AddData("max_support", SafetyCertifier::Get().GetMaxSupport());
    data_saver.AddData("removal_count", config_->removal_count_);
    data_saver.AddData("max_iterations", config_->max_iterations_);
    data_saver.AddData("risk", config_->risk_);
    data_saver.AddData("confidence", config_->confidence_);
    first_run_ = false;
  }

  data_saver.AddData("support_estimate", support_estimate_);
  data_saver.AddData("solver_iterations", iterations_);
  data_saver.AddData("removed_scenarios", removed_scenarios_);
  data_saver.AddData("failed_to_find_safe_solution", failed_to_find_safe_solution_);

  for (int i = 0; i < config_->max_iterations_; i++)
  {
    if (i < (int)cost_history_.size())
      data_saver.AddData("cost_" + std::to_string(i), cost_history_[i]);
    else
      data_saver.AddData("cost_" + std::to_string(i), -1);
  }

  // data_saver.AddData("cost", cost_history_.back());
}

int ScenarioConstraints::SimpleSequentialScenarioIterations(SolverInterface *solver_interface)
{
  int exit_code = -1;

  // Every iteration has variables (support, risk, cost, x*)
  for (int solve_iteration = 0; solve_iteration < config_->max_iterations_; solve_iteration++)
  {
    if (config_->debug_output_)
      ROS_INFO_STREAM("Scenario Main: Starting SQP Iteration " << solve_iteration);

    // We load the previous trajectory in the first iterations and reuse the previous iteration in later iterations
    if (solve_iteration == 0)                  // | feasible
      solver_interface->setReinitialize(true); // Initialize with the shifted trajectory
    else if (solve_iteration >= 1)
      solver_interface->setReinitialize(false);

    // Solve an iteration of the SQP problem and load the output
    exit_code = solver_interface->solve();
    solver_interface->printSolveInfo();
    cost_history_.push_back(solver_interface->Cost());
  }

  return exit_code;
}

void ScenarioConstraints::Visualize()
{
  LMPCC_INFO("Publishing scenario visuals");
  for (auto &disc : scenario_manager_)
    disc->Visualize();
}