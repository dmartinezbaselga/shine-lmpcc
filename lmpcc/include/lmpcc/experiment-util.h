#ifndef __EXPERIMENT_UTIL_H__
#define __EXPERIMENT_UTIL_H__

#include <ros_tools/data_saver.h>

#include <third_party/optuna.h>

#include <string>
#include <memory>

// Forward declarations
class predictive_configuration;
class RealTimeData;
class SolverInterface;
class VehicleRegion;

class Autotuner
{
public:
  Autotuner(int num_trials);

public:
  optuna::Trial &StartTrial();

  void FinalizeTrial(); // This should compute an objective value

  void PrintResults();

private:
  std::unique_ptr<optuna::Study> study_;
  optuna::SearchSpace search_space_;

  std::unique_ptr<optuna::Trial> current_trial_;

  int trial_count_ = 0;
  int num_trials_;
};

class ExperimentUtility
{
public:
  ExperimentUtility();
  void Init(predictive_configuration *config);

public:
  void Update(SolverInterface *solver, const RealTimeData &data, const VehicleRegion &vehicle);

  void OnTaskCompleted(bool objective_reached);

  void ForceDataSave();

  RosTools::DataSaver &GetDataSaver()
  {
    return data_saver_;
  };

private:
  void SaveData(SolverInterface *solver, const RealTimeData &data, const VehicleRegion &vehicle);
  void ExportData();

  // Data is saved in this object
  RosTools::DataSaver data_saver_;

  // When autuning, this utility manages the process
  std::unique_ptr<Autotuner> autotuner_;

  int experiment_counter_ = 0;
  int control_iteration_ = 0;
  int iteration_at_last_reset_ = 0; /* Track last reset point */

  predictive_configuration *config_;
};

// class SimulationTool
// {
// public:
//   SimulationTool(const std::string& topic, double min_time_between, int max_experiments)
//     : max_experiments_(max_experiments)
//   {
//     counter_ = 0;
//     finished_ = false;
//     reset_sub_ = nh_.subscribe(topic.c_str(), 1, &SimulationTool::ResetCallback, this);
//     timer_.reset(new RosTools::TriggeredTimer(min_time_between));
//     timer_->start();
//   }

// public:
//   void ResetCallback(const std_msgs::Empty& msg)
//   {
//     // Was this the last simulation (noting that the system is reset initially)
//     if (counter_ >= max_experiments_)
//     {
//       ROS_ERROR_STREAM("Simulation Tool: Done with " << max_experiments_ << " experiments!");
//       finished_ = true;
//     }

//     // Otherwise count
//     if (timer_->hasFinished())
//     {
//       counter_++;
//       ROS_WARN_STREAM("\033[34;47mSimulation Tool: === Experiment " << counter_ << " / " << max_experiments_
//                                                                     << " ===\033[m");
//     }
//   }

//   bool Finished() const
//   {
//     return finished_;
//   };

// private:
//   ros::NodeHandle nh_;
//   ros::Subscriber reset_sub_;

//   int counter_;
//   int max_experiments_;

//   bool finished_;

//   std::unique_ptr<RosTools::TriggeredTimer> timer_;
// };

#endif // __EXPERIMENT-UTIL_H__