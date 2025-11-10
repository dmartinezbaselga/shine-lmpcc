#include "interfaces/interface.h"

#include <lmpcc/lmpcc_controller.h>
#include <lmpcc/lmpcc_configuration.h>
#include <lmpcc_solver/SolverInclude.h>

Interface::Interface(ros::NodeHandle &nh, Controller *controller, predictive_configuration *config, SolverInterface *solver_interface_ptr)
{
  // Save pointers at init
  config_ = config;
  controller_ = controller;
  solver_interface_ptr_ = solver_interface_ptr;

  ros_markers_.reset(new RosTools::ROSMarkerPublisher(nh, "lmpcc/received_obstacles", config_->target_frame_, 20));

  external_velocity_set_ = false;

  data_.halfspaces_.resize(solver_interface_ptr->FORCES_N);
  for (size_t k = 0; k < data_.halfspaces_.size(); k++)
  {
    data_.halfspaces_[k].resize(config_->n_halfspaces_);
    for (int i = 0; i < config_->n_halfspaces_; i++)
      data_.halfspaces_[k][i] = RosTools::Halfspace::Dummy();
  }
}

void Interface::PostProcessObstacles(bool (*reject_function)(const Eigen::Vector2d &vehicle_pos,
                                                             const Eigen::Vector2d &obstacle_pos))
{
  LMPCC_INFO_STREAM("Post Processing " << data_.dynamic_obstacles_.size() << " Obstacles");

  // Create an index list
  std::vector<int> indices;
  indices.resize(data_.dynamic_obstacles_.size());
  std::iota(indices.begin(), indices.end(), 0);

  // If more, we sort and retrieve the closest obstacles
  if (data_.dynamic_obstacles_.size() > (unsigned int)config_->max_obstacles_)
  {
    std::vector<double> distances;
    LMPCC_INFO("\tReceived more obstacles than the maximum, ordering obstacles...");

    Eigen::Vector2d obstacle_pos;
    Eigen::Vector2d vehicle_pos(solver_interface_ptr_->State().x(), solver_interface_ptr_->State().y());

    distances.clear();
    for (auto &obstacle : data_.dynamic_obstacles_)
    {
      // Get the closest obstacles to the vehicle
      obstacle_pos(0) = obstacle.pose_.position.x;
      obstacle_pos(1) = obstacle.pose_.position.y;

      if (reject_function != nullptr && reject_function(vehicle_pos, obstacle_pos)) // If we should reject this
                                                                                    // obstacle, push a high distance
        distances.push_back(1e8);
      else
        distances.push_back(RosTools::dist(vehicle_pos, obstacle_pos));
    }

    // Sort obstacles on distance
    std::sort(indices.begin(), indices.end(), [&](const int a, const int b)
              { return (distances[a] < distances[b]); });

    // Keep the closest obstacles
    std::vector<DynamicObstacle> processed_obstacles;
    processed_obstacles.clear();

    for (int v = 0; v < config_->max_obstacles_; v++)
    {
      processed_obstacles.push_back(data_.dynamic_obstacles_[indices[v]]);
      processed_obstacles.back().discs_[0].id = v; // Make sure the disc_ids are sequential
    }

    data_.dynamic_obstacles_ = *(TrackedVector<DynamicObstacle> *)(&processed_obstacles);
  }
  else if (data_.dynamic_obstacles_.size() < (unsigned int)config_->max_obstacles_) /* If we did not receive enough, add
                                                                                       dummies */
  {
    LMPCC_INFO("\tReceived less obstacles than the maximum, adding dummies...");

    for (size_t cur_size = data_.dynamic_obstacles_.size(); cur_size < (unsigned int)config_->max_obstacles_;
         cur_size++)
    {
      // 0 radius discint id, int disc_start_id, const Eigen::Vector2d &pos, double width, double length, double
      // center_offset, int n_discs
      data_.dynamic_obstacles_.emplace_back(
          -1, cur_size,
          Eigen::Vector2d(solver_interface_ptr_->State().x() + 100. + (double)cur_size * 0.5,
                          solver_interface_ptr_->State().y() + 100. + (double)cur_size * 0.5),
          0.);

      data_.dynamic_obstacles_.back().pose_.position.x =
          solver_interface_ptr_->State().x() + 100. + (double)cur_size * 0.5;
      data_.dynamic_obstacles_.back().pose_.position.y =
          solver_interface_ptr_->State().y() + 100. + (double)cur_size * 0.5;
      data_.dynamic_obstacles_.back().PredictConstantVelocity(geometry_msgs::Twist(), solver_interface_ptr_->DT,
                                                              config_->N_pedestrians_);
    }
  }
  else
  {
    LMPCC_INFO("\tNo Post Processing Necessary");
  }

  LMPCC_INFO("\t" << data_.dynamic_obstacles_.size() << " Obstacles are ready for planning");
}

void Interface::PlotObstacles(RosTools::ROSMarkerPublisher *markers, bool plot_models, bool publish)
{
  RosTools::ROSPointMarker &obstacle_marker = markers->getNewPointMarker("CYLINDER");
  obstacle_marker.setScale(0.25, 0.25, 1.5);
  obstacle_marker.setColor(0.0, 0.0, 0.0, 0.8);
  double plot_height = 0.; // 1.3

  RosTools::ROSModelMarker &ped_model = markers->getNewModelMarker("package://lmpcc/models/walking.dae");

  // Plot all obstacles
  for (auto &object : data_.dynamic_obstacles_) // obstacle_msg_.objects)
  {
    geometry_msgs::Pose plot_pose = object.pose_;
    object.pose_.position.z = 1e-3 + plot_height / 2.;

    if (plot_models)
    {
      ped_model.setColorInt(object.id_, 1., RosTools::Colormap::BRUNO);
      ped_model.setOrientation(RosTools::quaternionToAngle(plot_pose.orientation) + M_PI_2); // For Carla (maybe remove
                                                                                             // later)
      ped_model.addPointMarker(plot_pose);
    }
    else
    {
      obstacle_marker.setScale(2.0 * config_->r_VRU_, 2.0 * config_->r_VRU_, plot_height);

      // This marker is a simple cube at the exact position.
      obstacle_marker.addPointMarker(plot_pose);
    }
  }

  if (publish)
    markers->publish();
}

void Interface::EmergencyStrategyConstantBraking(double deceleration)
{
  // Ensure positivity
  deceleration = 1.0 * std::abs(deceleration);
  double vel = controller_->velocity_; // solver_interface_->v();
  if (vel <= 0)                        // Don't start with negative velocity
    vel = 0.;

  // Copy the first state to our backup plan
  solver_interface_ptr_->Plan(0) = solver_interface_ptr_->State(); // Initial state is the current measurement
  solver_interface_ptr_->Plan(0).x() +=
      std::cos(solver_interface_ptr_->Plan(0).psi()) * vel * solver_interface_ptr_->DT;
  solver_interface_ptr_->Plan(0).y() +=
      std::sin(solver_interface_ptr_->Plan(0).psi()) * vel * solver_interface_ptr_->DT; // update with velocity

  double x, y;
  x = solver_interface_ptr_->Plan(0).x();
  y = solver_interface_ptr_->Plan(0).y();

  // Define a backup trajectory as constant deceleration
  for (size_t k = 1; k < solver_interface_ptr_->FORCES_N; k++)
  {
    double prev_vel = vel;
    vel -= deceleration * solver_interface_ptr_->DT; // v -= a*dt

    // No backwards driving
    if (vel <= 0.)
    {
      deceleration = prev_vel / solver_interface_ptr_->DT;
      vel = 0.;
    }

    // Update x, y, psi
    x += vel * std::cos(solver_interface_ptr_->State().psi()) * solver_interface_ptr_->DT;
    y += vel * std::sin(solver_interface_ptr_->State().psi()) * solver_interface_ptr_->DT;
    solver_interface_ptr_->Plan(k).set_x(x);
    solver_interface_ptr_->Plan(k).set_y(y);
    solver_interface_ptr_->Plan(k).set_psi(solver_interface_ptr_->State().psi());

    solver_interface_ptr_->a(k) = -deceleration;
    solver_interface_ptr_->spline(k) =
        solver_interface_ptr_->spline(k - 1) + vel * solver_interface_ptr_->DT; // Update the spline
  }
}

void Interface::PartitionCallback(const lmpcc_msgs::observation_partitioning &msg)
{
  monitor_->MarkReceived("Partitions");

  data_.obstacle_partitions_.clear();
  for (size_t i = 0; i < msg.object_ids.size(); i++)
  {
    // std::cout << msg.object_ids[i] << ", " << msg.partitions[i] << std::endl;
    data_.obstacle_partitions_[msg.object_ids[i]] = {
        msg.partitions[i], msg.observable_velocity[i]}; // Map from obstacle ID -> Partition
  }
}

void Interface::RoadmapWaypointsCallback(const nav_msgs::Path &msg)
{
  // if (config_->activate_debug_output_)
  LMPCC_INFO_STREAM("Interface: Received " << std::floor(msg.poses.size()) << " Waypoints");
  monitor_->MarkReceived("Waypoints");

  // If new waypoints are close to the old don't do anything
  // bool waypoints_are_the_same = data_.path_.Get().x_.size() == msg.poses.size();
  bool waypoints_are_the_same = true;
  for (size_t i = 0; i < std::floor(msg.poses.size()); i++)
  {
    if (std::sqrt(std::pow(msg.poses[i].pose.position.x - data_.path_.Get().x_[i], 2) +
                  std::pow(msg.poses[i].pose.position.y - data_.path_.Get().y_[i], 2)) > 1.0)
    {
      waypoints_are_the_same = false;
      break;
    }
  }

  // Do not update the reference path if the received path is the same (to reduce computation times)
  if (waypoints_are_the_same)
  {
    LMPCC_INFO("\tWaypoints are the same as the current plan, not updating");
    return;
  }
  ROS_WARN_STREAM("Interface: Received " << std::floor(msg.poses.size()) << " Waypoints");

  data_.path_.Get().Clear();

  for (size_t i = 0; i < msg.poses.size(); i++)
    data_.path_.Get().AddPose(msg.poses[i].pose);

  controller_->OnWaypointsReceived();
}

// Use Pedestrian Simulator instead
// void Interface::PedsimObstacleCallBack(const pedsim_msgs::AgentStates &msg)
// {
//   PROFILE_FUNCTION(); // So that we can see when it arrives

//   LMPCC_INFO("Interface::PedsimObstacleCallback received " << msg.agent_states.size() << " obstacles");
//   monitor_->MarkReceived("Obstacles");

//   data_.obstacles_received_time_ = msg.header.stamp;

//   // Load all obstacles into our format
//   data_.dynamic_obstacles_.clear();

//   int disc_counter = 0; // Track the number of obstacle discs

//   // Note: We loop until we have all the DISCS that we can handle
//   for (auto &agent_state : msg.agent_states)
//   {
//     // Pedestrians as a single disc
//     data_.dynamic_obstacles_.emplace_back(agent_state.id, disc_counter, Eigen::Vector2d(agent_state.pose.position.x, agent_state.pose.position.y), config_->r_VRU_);
//     // std::sqrt(//std::pow(object.shape.dimensions[0] / 2., 2.) + std::pow(object.shape.dimensions[1] / 2., 2.)));

//     data_.dynamic_obstacles_.back().pose_ = agent_state.pose;

//     // Todo!
//     data_.dynamic_obstacles_.back().PredictConstantVelocity(agent_state.twist, solver_interface_ptr_->DT, solver_interface_ptr_->FORCES_N);

//     disc_counter++;
//   }

//   LMPCC_INFO_STREAM("Interface: Converted " << data_.dynamic_obstacles_.size() << " obstacles to LMPCC format");

//   PlotObstacles(ros_markers_.get(), true, true);

//   PostProcessObstacles();

//   // // Load predictions into the obstacles
//   // for (auto &obstacle_prediction : msg.obstacles)
//   // {
//   //   for (auto &obstacle : data_.dynamic_obstacles_)
//   //   {
//   //     // Association based on id
//   //     if (obstacle_prediction.id == obstacle.id_)
//   //     {
//   //       obstacle.LoadPredictions(obstacle_prediction);
//   //       break;
//   //     }
//   //   }
//   // }

//   // Validate the predictions
//   if (config_->debug_output_)
//   {
//     for (auto &obstacle : data_.dynamic_obstacles_)
//     {
//       if (obstacle.prediction_.gaussians[0].mean.poses.size() == 0)
//       {
//         LMPCC_WARN("Predictions of Obstacle " << obstacle.id_ << " are empty!");
//       }
//     }
//   }

//   controller_->OnObstaclesReceived(); // Process in the modules if necessary
// }

void Interface::GoalCallback(const geometry_msgs::PoseStamped &msg)
{
  LMPCC_INFO_STREAM("Interface: Received a new goal (x = " << msg.pose.position.x << ", y = " << msg.pose.position.y << ")!")

  data_.goal_(0) = msg.pose.position.x;
  data_.goal_(1) = msg.pose.position.y;

  controller_->OnOtherDataReceived("Goal");
}

lmpcc_msgs::follower Interface::SendTrajectoryToFollower(SolverInterface *solver_interface)
{
  lmpcc_msgs::follower msg;
  msg.current_state.x = solver_interface->State().x();
  msg.current_state.y = solver_interface->State().y();
  msg.current_state.psi = solver_interface->State().psi();
  msg.current_state.v = solver_interface->State().v();
  msg.current_state.a = solver_interface->control_inputs_.acceleration;

  for (size_t k = 0; k < solver_interface->FORCES_N; k++)
  {
    lmpcc_msgs::follower_state cur_state;
    cur_state.x = solver_interface->Plan(k).x();
    cur_state.y = solver_interface->Plan(k).y();
    cur_state.psi = solver_interface->Plan(k).psi();
    cur_state.v = solver_interface->Plan(k).v();
    cur_state.a = solver_interface->a(k);
    msg.trajectory.push_back(cur_state);
  }

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";

  return msg;
}