#include "modules_constraints/linearized_constraints.h"

#include <lmpcc/lmpcc_configuration.h>
#include <lmpcc_solver/SolverInclude.h>

LinearizedConstraints::LinearizedConstraints(ros::NodeHandle &nh, predictive_configuration *config,
                                             VehicleRegion *vehicle)
    : ControllerModule(nh, config, vehicle)
{
  type_ = ModuleType::CONSTRAINT;

  LMPCC_INFO("Initializing LinearizedConstraints Module")

  // Initialise the visualisation
  ros_markers_.reset(new RosTools::ROSMarkerPublisher(nh, "lmpcc/linearized_constraints/markers", config_->target_frame_, 200));
  ros_static_markers_.reset(
      new RosTools::ROSMarkerPublisher(nh, "lmpcc/linearized_constraints/static", config_->target_frame_, 20));

  a1_.resize(vehicle->discs_.size());
  a2_.resize(vehicle->discs_.size());
  b_.resize(vehicle->discs_.size());
  for (size_t d = 0; d < vehicle->discs_.size(); d++)
  {
    // Initialize arrays for the constraints
    a1_[d].resize(predictive_configuration::N);
    a2_[d].resize(predictive_configuration::N);
    b_[d].resize(predictive_configuration::N);
    for (size_t k = 0; k < predictive_configuration::N; k++)
    {
      a1_[d][k] = Eigen::ArrayXd(config->max_obstacles_);
      a2_[d][k] = Eigen::ArrayXd(config->max_obstacles_);
      b_[d][k] = Eigen::ArrayXd(config->max_obstacles_);
    }
  }

  num_obstacles_ = 0;
}

void LinearizedConstraints::Update(SolverInterface *solver_interface, RealTimeData &data)
{
  obstacles_ = (std::vector<DynamicObstacle> *)(&data.dynamic_obstacles_);

  std::vector<DynamicObstacle> copied_obstacles = data.dynamic_obstacles_;
  num_obstacles_ = copied_obstacles.size();
  data_ptr_ = &data;

  if (copied_obstacles.size() == 0)
    return;

  for (auto &obstacle : copied_obstacles)
  {
    for (auto &disc : obstacle.discs_)
      disc.radius = 1e-2; // We fully relax the constraints here
  }

  auto &vehicle_prediction = solver_interface->InitialVehiclePrediction();
  // For all stages
  for (size_t k = 0; k < solver_interface->FORCES_N; k++)
  {
    for (size_t disc_id = 0; disc_id < vehicle_prediction[k].discs_.size(); disc_id++)
    {
      Disc &disc = vehicle_prediction[k].discs_[disc_id];
      Eigen::Vector2d pos = vehicle_prediction[k].discs_[disc_id].AsVector2d(); // vehicle_->discs_[0].poses_[k];

      // Ensure that the vehicle position is collision-free
      ProjectToSafety(k, pos, copied_obstacles, solver_interface->area_->DiscRadius());

      // If we projected, load the updated position back into the solver
      disc.SetPosition(pos);                                                                         // Save the projected position in the Disc
      Eigen::Vector2d associated_vehicle_pos = disc.TranslateToVehicleCenter(vehicle_prediction[k]); // Translate the disc to the vehicle position
      vehicle_prediction[k].SetPosition(associated_vehicle_pos);                                     // Save the vehicle position
      solver_interface->LoadVehiclePredictionsToInitialPlan(vehicle_prediction);

      // For all obstacles
      for (size_t obs_id = 0; obs_id < copied_obstacles.size(); obs_id++)
      {
        auto &obstacle_prediction = copied_obstacles[obs_id].GetPredictedCollisionRegions();

        int k_obs;
        if (config_->synchronized_actuation_)
          k_obs = k == solver_interface->FORCES_N - 1 ? (int)solver_interface->FORCES_N - 1 : k + 1; // one step forward
        else
          k_obs = k;

        Eigen::Vector2d obstacle_pos = obstacle_prediction[k_obs].AsVector2d();

        double diff_x = obstacle_pos(0) - pos(0);
        double diff_y = obstacle_pos(1) - pos(1);

        double d = (obstacle_pos - pos).norm();

        // Compute the components of A for this obstacle (normalized normal vector)
        a1_[disc_id][k](obs_id) = diff_x / d;
        a2_[disc_id][k](obs_id) = diff_y / d;

        // Compute b (evaluate point on the collision circle)
        b_[disc_id][k](obs_id) = a1_[disc_id][k](obs_id) * obstacle_pos(0) + a2_[disc_id][k](obs_id) * obstacle_pos(1) -
                                 (obstacle_prediction[k_obs].radius +
                                  solver_interface->area_->DiscRadius()); // vehicle_prediction[k].discs_[0].radius);
      }
    }
  }
}

void LinearizedConstraints::ProjectToSafety(int k, Eigen::Vector2d &pos, std::vector<DynamicObstacle> &obstacles,
                                            double vehicle_radius)
{
  if (obstacles.size() == 0)
    return;

  // Project to a collision free position if necessary, considering all the obstacles
  for (int iterate = 0; iterate < 3; iterate++) // At most iterates iterations
  {
    for (auto &obstacle : obstacles)
    {
      auto &obstacle_prediction = obstacle.GetPredictedCollisionRegions();

      pos = dr_projection_.DouglasRachfordProjection(pos, obstacle_prediction[k].AsVector2d(),
                                                     obstacles[0].discs_[0].AsVector2d(),
                                                     obstacle.discs_[0].radius + vehicle_radius, pos);
    }
  }
}

void LinearizedConstraints::SetParameters(SolverInterface *solver_interface, const RealTimeData &data, int k_solver,
                                          int &param_idx)
{

  assert(k_solver > 0);
  // Compensate for plan displacement!
  int k = k_solver - 1; // Because the initial state is skipped, we need to do "-1"

  if (k == 0)
  {
    LMPCC_INFO("Linearized Obstacles: Setting Solver Parameters ("
               << num_obstacles_ << " obstacles and " << config_->max_obstacles_ - num_obstacles_ << " Dummies)");
  }

  for (size_t disc_id = 0; disc_id < vehicle_->discs_.size(); disc_id++)
  {
    int l = 0;

    for (; l < num_obstacles_; l++)
    {
      solver_interface->setParameter(k_solver, param_idx, a1_[disc_id][k](l));
      solver_interface->setParameter(k_solver, param_idx, a2_[disc_id][k](l));
      solver_interface->setParameter(k_solver, param_idx, b_[disc_id][k](l));
    }

    // Static obstacles!
    int j = 0;
    for (; j < std::min(config_->n_halfspaces_, (int)data.halfspaces_.size()); j++)
    {

      const auto &halfspace = data.halfspaces_[k][j];
      solver_interface->setParameter(k_solver, param_idx, halfspace.A_(0));
      solver_interface->setParameter(k_solver, param_idx, halfspace.A_(1));
      solver_interface->setParameter(k_solver, param_idx, halfspace.b_);
    }
    l += j;

    // Fill the remainder with dummies
    for (; (int)l < config_->max_obstacles_ + config_->n_halfspaces_; l++)
    {
      solver_interface->setParameter(k_solver, param_idx, 1.);
      solver_interface->setParameter(k_solver, param_idx, 0.);
      solver_interface->setParameter(k_solver, param_idx, solver_interface->InitialPlan(k).x() + 100.);
    }
  }
}

//-------------------------------- VISUALIZATION -----------------------------------//
void LinearizedConstraints::Visualize()
{
  LMPCC_WARN("Visualizing Linearized Constraints");

  if (data_ptr_ == nullptr)
    return;

  RosTools::ROSLine &line = ros_markers_->getNewLine();
  RosTools::ROSLine &static_line = ros_static_markers_->getNewLine();

  for (size_t draw_idx = 0; draw_idx < config_->indices_to_draw_.size(); draw_idx++)
  {
    int k = config_->indices_to_draw_[draw_idx];

    // Draw dynamic constraints
    line.setColorInt((int)draw_idx, (int)config_->indices_to_draw_.size(), 1.0);
    line.setScale(0.15, 0.15);

    for (int l = 0; l < num_obstacles_; l++)
      RosTools::DrawLine(line, a1_[0][k](l), a2_[0][k](l), b_[0][k](l), 40.);

    // Draw static constraints
    static_line.setColorInt(0, (int)config_->indices_to_draw_.size(), 1.0);
    static_line.setScale(0.05, 0.05);
    for (int j = 0; j < std::min(config_->n_halfspaces_, (int)data_ptr_->halfspaces_.size()); j++)
    {
      const auto &halfspace = data_ptr_->halfspaces_[k][j];
      RosTools::DrawLine(static_line, halfspace.A_(0), halfspace.A_(1), halfspace.b_, 40.);
    }
  }

  ros_markers_->publish();
  ros_static_markers_->publish();
}

int LinearizedConstraints::NumActiveConstraints(SolverInterface *solver_interface)
{
  int active = 0;
  auto &vehicle_regions = solver_interface->OptimizedVehiclePredictions(); // Get the ego-vehicle predictions
  for (size_t k = 0; k < vehicle_regions.size(); k++)
  {
    Eigen::Vector2d pose = vehicle_regions[k].discs_[0].AsVector2d();
    for (int l = 0; l < num_obstacles_; l++)
    {
      double val = a1_[0][k](l) * pose(0) + a2_[0][k](l) * pose(1) - b_[0][k](l);

      if (val > -1e-5 && val < 1e-8)
        active++;
    }
  }

  return active;
}