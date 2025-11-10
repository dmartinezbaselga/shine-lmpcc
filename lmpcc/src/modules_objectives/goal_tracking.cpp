#include "modules_objectives/goal_tracking.h"

#include <lmpcc/lmpcc_configuration.h>
#include <lmpcc_solver/SolverInclude.h>

GoalTracking::GoalTracking(ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle)
    : ControllerModule(nh, config, vehicle)
{
  LMPCC_WARN_ALWAYS("Initializing Goal Tracking Module");

  type_ = ModuleType::OBJECTIVE;

  goal_received_ = false;
  goal_not_received_msg_ = false;

  // Reference Path
  ros_markers_.reset(new RosTools::ROSMarkerPublisher(nh, "lmpcc/goal", config_->target_frame_, 5));

  LMPCC_WARN_ALWAYS("Goal Tracking Module Initialized");
}

bool GoalTracking::ReadyForControl(SolverInterface *solver_interface, const RealTimeData &data)
{
  if (!goal_received_ && !goal_not_received_msg_)
  {
    LMPCC_WARN("Waiting for the goal");
    goal_not_received_msg_ = true;
  }
  return goal_received_;
}

bool GoalTracking::ObjectiveReached(SolverInterface *solver_interface, const RealTimeData &data)
{
  return goal_reached_ && !first_run_;
}

void GoalTracking::Update(SolverInterface *solver_interface, RealTimeData &data)
{
  PROFILE_AND_LOG(config_->debug_output_, "GoalTracking::Update");
  first_run_ = false;

  // Check if the end of the path was reached
  Eigen::Vector2d pos(solver_interface->State().x(), solver_interface->State().y());
  if (RosTools::dist(goal_, pos) < 0.5)
  {
    if (!goal_reached_)
    {
      LMPCC_INFO("Reached the goal. Please supply a new goal!");
      goal_reached_ = true;
    }
  }
}

void GoalTracking::OnReset(SolverInterface *solver_interface)
{
  // goal_received_ = false;
  // goal_ = Eigen::Vector2d(0., 0.);

  first_run_ = true;
  goal_reached_ = false;
  // goal_not_received_msg_ = false;
}

void GoalTracking::OnDataReceived(SolverInterface *solver_interface, RealTimeData &data, std::string &&data_name)
{
  if (data_name == "Goal")
  {
    LMPCC_WARN("GoalTracking: Received a new goal");

    goal_ = data.goal_;
    goal_received_ = true;
    goal_not_received_msg_ = false;
  }
}

void GoalTracking::Visualize()
{
  LMPCC_INFO("GoalTracking: Updating Visuals");
  RosTools::ROSPointMarker &goal = ros_markers_->getNewPointMarker("CUBE");

  goal.setColorInt(0, 1., RosTools::Colormap::BRUNO);
  goal.setScale(0.5, 0.5, 0.05);

  if (goal_received_)
    goal.addPointMarker(Eigen::Vector3d(goal_(0), goal_(1), 0.));

  ros_markers_->publish();
}

void GoalTracking::SetParameters(SolverInterface *solver_interface, const RealTimeData &data, int N_iter,
                                 int &param_idx)
{
  // Add the goal
  solver_interface->setParameter(N_iter, param_idx, goal_(0));
  solver_interface->setParameter(N_iter, param_idx, goal_(1));
}