#include "lmpcc/experiment-util.h"

#include <lmpcc/lmpcc_configuration.h>
#include <lmpcc_solver/SolverInclude.h>
#include <lmpcc/types.h>
#include <lmpcc_tools/collision_region.h>

#include <ros_tools/profiling.h>

#include <boost/filesystem.hpp>

using namespace optuna;

Autotuner::Autotuner(int num_trials)
{
  num_trials_ = num_trials;
  study_.reset(new Study("sqlite:///example.db", "test_study2", optuna::MINIMIZE,
                         true)); // The name "test_study2" defines where trials are added!

  // Define parameters to be tuned
  search_space_.add_float("x", -10, 10);
}

optuna::Trial &Autotuner::StartTrial()
{
  current_trial_.reset(new Trial(study_->ask(search_space_))); // Ask the search space for new parameters
  return *current_trial_;
}

void Autotuner::FinalizeTrial() // Todo: argument
{
  double objective_value = current_trial_->param<double>("x");
  study_->tell(*current_trial_, objective_value); // Finalizes the TRIAL with the given result!
  trial_count_++;

  if (trial_count_ >= num_trials_)
  {
    LMPCC_SUCCESS_ALWAYS("Autuning completed (" << num_trials_ << " trials)");
    PrintResults();
  }
}

void Autotuner::PrintResults()
{
  // For all trials
  for (const optuna::FrozenTrial &trial : study_->trials())
  {
    // Report the results
    std::cout << trial.number << " " << trial.state << " "
              << " " << trial.param<double>("x") << " " << trial.value << std::endl;
  }

  // Report on the best trial
  const optuna::FrozenTrial best_trial = study_->best_trial();
  std::cout << best_trial.number << " " << best_trial.value << std::endl;
}

ExperimentUtility::ExperimentUtility() {}

void ExperimentUtility::Init(predictive_configuration *config)
{
  config_ = config;

  LMPCC_WARN_ALWAYS("Recording with name: " << boost::filesystem::weakly_canonical(config_->recording_name_));

  // autotuner_.reset(new Autotuner(10));

  // // Largely from: https://github.com/not522/optuna-cpp/blob/main/example.cpp
  // if (true)  // Replace with setting
  // {
  //   // For a number of trials
  //   for (int i = 0; i < 10; ++i)
  //   {
  //     Trial& trial = autotuner_->StartTrial();
  //     autotuner_->FinalizeTrial();
  //   }
  // }
}

void ExperimentUtility::Update(SolverInterface *solver, const RealTimeData &data, const VehicleRegion &vehicle)
{
  // Save data of this control iteration
  SaveData(solver, data, vehicle);

  // Save data to FILE when time based recording is enabled
  if (config_->record_on_time_ && control_iteration_ % config_->save_every_x_iterations_ == 2)
    data_saver_.SaveData(config_->recording_name_);
}

void ExperimentUtility::ForceDataSave()
{
  data_saver_.SaveData(config_->recording_name_);
}

void ExperimentUtility::OnTaskCompleted(bool objective_reached)
{
  // ADD DATA OF THIS EXPERIMENT
  // Add the control iteration where the reset was triggered - This divides the saved data!
  data_saver_.AddData("reset", control_iteration_);

  // Add the duration
  data_saver_.AddData("metric_duration", (control_iteration_ - iteration_at_last_reset_) * (1.0 / config_->clock_frequency_));
  data_saver_.AddData("metric_completed", (int)(objective_reached));
  iteration_at_last_reset_ = control_iteration_;

  // Save data to FILE when a number of experiments have been completed
  if ((!config_->record_on_time_) && (experiment_counter_ % config_->save_every_x_experiments_ == 0) && experiment_counter_ > 0)
    data_saver_.SaveData(config_->recording_name_); // This saves the data to file

  experiment_counter_++;

  // Save profiling data before we crash the controller
  if (experiment_counter_ >= config_->number_of_experiments_)
    RosTools::Instrumentor::Get().EndSession();

  ROSTOOLS_ASSERT(experiment_counter_ < config_->number_of_experiments_ + 1, "Configured number of experiments, completed.");
}

void ExperimentUtility::SaveData(SolverInterface *solver, const RealTimeData &data, const VehicleRegion &vehicle)
{
  LMPCC_INFO("ExperimentUtility::SaveData()");

  // Don't export if the obstacles aren't ready
  if (data.dynamic_obstacles_.size() == 0)
  {
    LMPCC_INFO("Not exporting data: Obstacles not yet received.");
    return;
  }

  // SAVE VEHICLE DATA
  data_saver_.AddData("vehicle_pose", vehicle.pos_);
  data_saver_.AddData("vehicle_orientation", vehicle.orientation_);

  // Get the vehicle predictions
  auto &vehicle_regions = solver->InitialVehiclePrediction();

  // Save disc poses
  if (vehicle_regions.size() > 0)
  {
    for (auto &disc : vehicle_regions[0].discs_)
    {
      data_saver_.AddData("vehicle_" + std::string(disc) + "_pose",
                          disc.AsVector2d()); // disc.TranslateToDisc(vehicle_pose, vehicle_orientation));
    }
  }

  for (size_t k = 0; k < vehicle_regions.size(); k++)
  {
    data_saver_.AddData("vehicle_plan_" + std::to_string(k), Eigen::Vector2d(solver->Plan(k).x(), solver->Plan(k).y()));
  }

  // ADD SOLVER DATA

  // --- ADD RECEIVED DATA (from the interface) --- //
  // std::cout << "checking collisions...\n"
  //           << "State received " << (ros::Time::now() - data.state_received_time_).nsec / 1.0e6 << "ms ago\n "
  //           << "Obstacles received " << (ros::Time::now() - data.obstacles_received_time_).nsec / 1.0e6 << "ms ago" << std::endl;

  // OBSTACLES
  // int collisions = 0;
  // double max_intrusion = 0.;
  for (size_t v = 0; v < data.dynamic_obstacles_.size(); v++)
  {
    auto &obstacle = data.dynamic_obstacles_[v];

    // POSE AND ORIENTATION
    Eigen::Vector2d pose(obstacle.pose_.position.x, obstacle.pose_.position.y); // Can be outdated!
    double orientation = RosTools::quaternionToAngle(obstacle.pose_);

    // Jackal simulator
    // data_saver_.AddData("obstacle_" + std::to_string(v) + "_pose", pose);
    // data_saver_.AddData("obstacle_" + std::to_string(v) + "_orientation", orientation);

    // CARLA / Real Jackal
    if (obstacle.id_ != -1)
    {
      data_saver_.AddData("obstacle_map_" + std::to_string(v), obstacle.id_);
      data_saver_.AddData("obstacle_" + std::to_string(obstacle.id_) + "_pose", pose);
      data_saver_.AddData("obstacle_" + std::to_string(obstacle.id_) + "_orientation", orientation);
    }

    // DISCS
    for (auto &disc : obstacle.discs_)
    {
      data_saver_.AddData("disc_" + std::to_string(disc.id) + "_pose", disc.AsVector2d());
      data_saver_.AddData("disc_" + std::to_string(disc.id) + "_radius", disc.radius); // Write with the correct id?
      data_saver_.AddData("disc_" + std::to_string(disc.id) + "_obstacle", v);
    }

    // // COLLISIONS
    // if ((vehicle.pos_ - pose).norm() < obstacle.discs_[0].radius + vehicle.discs_[0].radius - 1e-2)
    // {
    //   double intrusion = -((vehicle.pos_ - pose).norm() - obstacle.discs_[0].radius - vehicle.discs_[0].radius);
    //   LMPCC_WARN_ALWAYS("Collision Detected. Intrusion: " << intrusion << "m");
    //   collisions++;
    //   max_intrusion = std::max(intrusion, max_intrusion);
    // }
  }
  data_saver_.AddData("max_intrusion", data.intrusion_);
  data_saver_.AddData("metric_collisions", int(data.intrusion_ > 0.));

  // TIME KEEPING
  data_saver_.AddData("iteration", control_iteration_);
  control_iteration_++;
}