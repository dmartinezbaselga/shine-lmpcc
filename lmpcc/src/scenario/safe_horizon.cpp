#include "scenario/safe_horizon.h"
#include "lmpcc_solver/SolverInclude.h"

SafeHorizon::SafeHorizon(ros::NodeHandle &nh, predictive_configuration *config, int disc_id, GaussianSampler &sampler, bool enable_visualization = false)
    : config_(config), enable_visualization_(enable_visualization), disc_id_(disc_id)
{
  LMPCC_WARN_ALWAYS("SH-MPC: Initializing disc...");

  // Initialise the visualisation
  if (enable_visualization)
    ros_markers_.reset(new RosTools::ROSMarkerPublisher(nh, "scenario_constraints/markers", config_->target_frame_, 500)); // 3500)); // was 1800

  sampler_ = &sampler;

  // Save some useful variables
  S = SafetyCertifier::Get().GetSampleSize();
  N = predictive_configuration::N;

  // Resize over the horizon
  verify_poses_.resize(N); // Active constraints checking
  scenario_threads_.resize(N);
  old_intersects_.resize(N);

  // Keeping track of the location of infeasible scenarios per k
  infeasible_scenario_poses_.resize(N);
  infeasible_scenario_idxs_.resize(N);
  for (size_t k = 0; k < infeasible_scenario_poses_.size(); k++)
  {
    infeasible_scenario_poses_[k].reserve(20);
    infeasible_scenario_idxs_[k].reserve(20);
  }

  // Dynamic + Range + Static
  int constraint_size = S * config_->max_obstacles_ + 4 + config_->n_halfspaces_;

  // Arrays for constraint construction
  diffs_x_.resize(N);
  diffs_y_.resize(N);
  distances_.resize(N);

  a1_.resize(N);
  a2_.resize(N);
  b_.resize(N);
  scenario_indices_.resize(N);

  x_left_.resize(N);
  x_right_.resize(N);

  for (size_t k = 0; k < N; k++)
  {
    // Cosntraint size!
    diffs_x_[k] = Eigen::ArrayXd(S);
    diffs_y_[k] = Eigen::ArrayXd(S);
    distances_[k] = Eigen::ArrayXd(S);

    // Constraint variables
    a1_[k] = Eigen::ArrayXd(constraint_size);
    a2_[k] = Eigen::ArrayXd(constraint_size);
    b_[k] = Eigen::ArrayXd(constraint_size);

    // Populate the scenario indices
    scenario_indices_[k].resize(constraint_size);

    // Dynamic
    for (int v = 0; v < config_->max_obstacles_; v++)
    {
      scenario_indices_[k].resize(S);
      for (int s = 0; s < (int)S; s++)
        scenario_indices_[k][v * S + s] = Scenario{s, v};
    }

    // Range / Static
    int index;
    for (int i = 0; i < config_->n_halfspaces_ + 4; i++)
    {
      index = config_->max_obstacles_ * S + i;
      scenario_indices_[k][index] = Scenario{index, -1};
    }
  }

  // Reserve space for the constraints and the y evaluations
  constraints_.resize(N);
  y_left_.resize(N);
  y_right_.resize(N);

  for (size_t k = 0; k < N; k++)
  {
    y_left_[k] = Eigen::ArrayXd(constraint_size);
    y_right_[k] = Eigen::ArrayXd(constraint_size);

    // Populate the constraints
    constraints_[k].reserve(constraint_size);

    // Dynamic constraints
    for (int v = 0; v < config_->max_obstacles_; v++)
    {
      for (size_t s = 0; s < S; s++)
      {
        constraints_[k].emplace_back(&scenario_indices_[k][v * S + s], ObstacleType::DYNAMIC, ConstraintSide::UNDEFINED);
      }
    }

    // External static constraints
    for (int i = 0; i < config_->n_halfspaces_; i++)
      constraints_[k].emplace_back(&scenario_indices_[k][config_->max_obstacles_ * S + i], ObstacleType::STATIC, ConstraintSide::UNDEFINED);

    // Range constraints
    for (size_t i = 0; i < 4; i++)
      constraints_[k].emplace_back(&scenario_indices_[k][config_->max_obstacles_ * S + config_->n_halfspaces_ + i], ObstacleType::RANGE, ConstraintSide::UNDEFINED);
  }

  // Initialize polygon constructors
  for (u_int k = 0; k < N; k++)
  {
    polytopes_.emplace_back(&constraints_[k], &y_left_[k], &y_right_[k], constraint_size, config_->polygon_range_);
  }

  polygon_areas_.resize(N);
  for (u_int k = 0; k < N; k++)
    polygon_areas_[k] = 0.0;

  // Variable initialisation
  radii_.resize(config_->max_obstacles_);

  // Support subsample (size arguments are not critical)
  support_subsample_.support_indices_.resize(50);
  support_subsample_.scenarios_.resize(50);
  removed_scenarios_.support_indices_.resize(config_->removal_count_);
  removed_scenarios_.scenarios_.resize(config_->removal_count_);

  LMPCC_WARN_ALWAYS("\tSH-MPC: Done!");
}

void SafeHorizon::LoadData(SolverInterface *solver_interface, RealTimeData &data)
{
  LMPCC_INFO("Safe Horizon: LoadData()");

  clearAll(); // Clear data from previous runs

  status_ = ScenarioStatus::SUCCESS;
  is_feasible_ = true;

  // Retrieve trajectory scenarios
  scenarios_ = sampler_->GetSamples();
  LMPCC_INFO("Safe Horizon: Scenarios Retrieved");

  // Save a pointer to the obstacles
  obstacles_ = (std::vector<DynamicObstacle> *)(&data.dynamic_obstacles_); // Save a pointer to the obstacles
  copied_obstacles_ = data.dynamic_obstacles_;                             // Thread safe

  // Store the radii of all obstacles
  int disc_id = 0;
  for (size_t v = 0; v < copied_obstacles_.size(); v++)
  {
    for (size_t d = 0; d < copied_obstacles_[v].discs_.size(); d++)
    {
      radii_[disc_id] = copied_obstacles_[v].discs_[d].radius + config_->vehicle_width_ / 2.;
      disc_id++;
    }
  }

  // Retrieve the positions of the current plan
  plan_ = solver_interface->InitialVehiclePrediction();

  // Project the plan
  PushAlgorithm(solver_interface);                  // Orthogonal push
  DRProjection(solver_interface, data.halfspaces_); // Douglas-Rachford splitting

  // Is this plan feasible now?
  for (size_t k = 0; k < N; k++)
  {
    for (int obst_id = 0; obst_id < config_->max_obstacles_; obst_id++) // For all obstacles
    {
      computeDistances(k, obst_id); // Computes for the scenarios of this obstacle the distances to the vehicle

      checkFeasibilityByDistance(k, obst_id); // We can then check if the scenarios are all feasible
    }
  }

  if (is_feasible_)
    return;

  std::vector<double> deceleration_list = {-0.2, 0.0, 0.5};

  for (double deceleration : deceleration_list)
  {
    is_feasible_ = true;

    useBackupPlan(solver_interface, velocity_, deceleration);

    // Project the plan
    PushAlgorithm(solver_interface);                  // Ad-hoc push
    DRProjection(solver_interface, data.halfspaces_); // Douglas-Rachford splitting
    for (size_t k = 0; k < N; k++)
    {
      for (int obst_id = 0; obst_id < config_->max_obstacles_; obst_id++) // For all obstacles
      {
        computeDistances(k, obst_id); // Computes for the scenarios of this obstacle the distances to the vehicle

        checkFeasibilityByDistance(k, obst_id); // We can then check if the scenarios are all feasible
      }
    }

    if (is_feasible_) // If we are feasible now, it makes sense to update the initial guess as well
    {
      solver_interface->LoadVehiclePredictionsToInitialPlan(plan_);
      return;
    }
  }
}

// Todo: dynamic_obstacles should just be a pointer
void SafeHorizon::Update(SolverInterface *solver_interface, RealTimeData &data)

{
  PROFILE_AND_LOG(config_->debug_output_, "Safe Horizon: Update");

  LoadData(solver_interface, data);

  // We need to have a feasible plan here
  LMPCC_INFO("Computing Distances");
  for (size_t k = 0; k < N; k++)
  {
    for (int obst_id = 0; obst_id < config_->max_obstacles_; obst_id++) // For all obstacles
    {
      computeDistances(k, obst_id); // Computes for the scenarios of this obstacle the distances to the vehicle

      checkFeasibilityByDistance(k, obst_id); // We can then check if the scenarios are all feasible

      computeHalfspaces(k, obst_id); // Then, we construct halfspaces for all scenarios
    }

    // Finally we intersect all constraints for all obstacles to obtain a safe polytope
    constructPolytopes(k, data.halfspaces_[k]);

    polygon_areas_[k] = polytopes_[k].getArea();
  }

  if (!is_feasible_)
    status_ = ScenarioStatus::INFEASIBLE;

  LMPCC_INFO_STREAM("SH-MPC: Update Done! Status: " << (int)status_);
}

// Clear data from previous iterations
void SafeHorizon::clearAll()
{

  LMPCC_INFO("SH-MPC: Clear All");
  for (size_t k = 0; k < N; k++)
    old_intersects_[k].clear();

  removed_scenarios_.Reset();
  support_subsample_.Reset();

  // Reset polygon computations
  for (size_t k = 0; k < N; k++)
    polytopes_[k].Reset();

  // Feasibility variables
  distances_feasible_ = std::vector<bool>(N, true);

  for (size_t k = 0; k < N; k++)
  {
    infeasible_scenario_poses_[k].clear();
    infeasible_scenario_idxs_[k].clear();
  }
}

void SafeHorizon::computeDistances(int k, int obstacle_id)
{
  Eigen::Vector2d pose = plan_[k].discs_[disc_id_].AsVector2d();

  diffs_x_[k] = (*scenarios_)[k][obstacle_id][0].array() - pose(0); // Note that the samples are already in a data structure that is fast to handle here
  diffs_y_[k] = (*scenarios_)[k][obstacle_id][1].array() - pose(1);

  distances_[k] = (diffs_x_[k] * diffs_x_[k] + diffs_y_[k] * diffs_y_[k]).sqrt();
}

void SafeHorizon::checkFeasibilityByDistance(int k, int obstacle_id)
{

  // Check for all samples if they are feasible
  for (int s = 0; s < distances_[k].size(); s++)
  {
    if (distances_[k](s) < radii_[obstacle_id]) // If distance to scenario < r, then infeasible
    {
      infeasible_scenario_poses_[k].push_back(Eigen::Vector2d((*scenarios_)[k][obstacle_id][0](s), (*scenarios_)[k][obstacle_id][1](s)));

      infeasible_scenario_idxs_[k].push_back(obstacle_id * S + s);

      distances_feasible_[k] = false;
      is_feasible_ = false;
    }
  }
}

void SafeHorizon::computeHalfspaces(int k, int obstacle_id)
{

  // Compute the components of A for this obstacle (normalized normal vector)
  a1_[k].block(obstacle_id * S, 0, S, 1) = diffs_x_[k] / distances_[k];
  a2_[k].block(obstacle_id * S, 0, S, 1) = diffs_y_[k] / distances_[k];

  // Compute b (evaluate point on the collision circle)
  b_[k].block(obstacle_id * S, 0, S, 1) =
      a1_[k].block(obstacle_id * S, 0, S, 1) * (*scenarios_)[k][obstacle_id][0].array() + a2_[k].block(obstacle_id * S, 0, S, 1) * (*scenarios_)[k][obstacle_id][1].array() - radii_[obstacle_id];
}

void SafeHorizon::constructPolytopes(int k, const std::vector<RosTools::Halfspace> &halfspaces)
{
  // Add range constraints
  auto &disc = plan_[k].discs_[disc_id_];
  polytopes_[k].AddRangeConstraints(disc.AsVector2d(), plan_[k].orientation_, a1_[k], a2_[k], b_[k]);

  // Add external halfspace constraints
  double a0_ext, a1_ext, b_ext;
  for (int i = 0; i < config_->n_halfspaces_; i++)
  {
    // Guard against 0 constraints by slightly altering the constraints
    a0_ext = halfspaces[i].A_(0);
    if (std::abs(a0_ext) < 1e-4)
      a0_ext = 1e-4;

    a1_ext = halfspaces[i].A_(1);
    if (std::abs(a1_ext) < 1e-4)
      a1_ext = 1e-4;

    b_ext = halfspaces[i].b_;

    a1_[k](config_->max_obstacles_ * S + i) = a0_ext;
    a2_[k](config_->max_obstacles_ * S + i) = a1_ext;
    b_[k](config_->max_obstacles_ * S + i) = b_ext;
  }

  // Compute the extreme search x's for each vehicle position
  x_left_[k] = disc.x - 1e-8 - std::sqrt(2) * config_->polygon_range_;
  x_right_[k] = disc.x + 1e-8 + std::sqrt(2) * config_->polygon_range_;

  // Compute all y at the left and the right for this k
  y_left_[k] = (b_[k] - a1_[k] * x_left_[k]) / a2_[k];
  y_right_[k] = (b_[k] - a1_[k] * x_right_[k]) / a2_[k];

  // Assign the sides of the constraints based on a2
  for (size_t c = 0; c < constraints_[k].size(); c++)
    constraints_[k][c].side_ = a2_[k](c) > 0 ? ConstraintSide::TOP : ConstraintSide::BOTTOM;

  // Construct the polytopes
  bool success;
  success = polytopes_[k].Search(disc.AsVector2d(), plan_[k].orientation_, x_left_[k], x_right_[k]);

  if (!success)
    polygon_failed_ = true;

  // Polytopes are stored in polytopes_[k]
}

void SafeHorizon::DRProjection(SolverInterface *solver_interface, const std::vector<std::vector<RosTools::Halfspace>> &halfspaces)
{
  int iterates = 5;

  for (size_t k = 0; k < solver_interface->FORCES_N; k++) // For each k
  {
    auto &disc = plan_[k].discs_[disc_id_];
    Eigen::Vector2d pose = disc.AsVector2d();                                                // Pose of this disc
    Eigen::Vector2d start_pose = k == 0 ? pose : plan_[k - 1].discs_[disc_id_].AsVector2d(); // Start pose refers to the previous pose
    Eigen::Vector2d prev_pose = pose;

    for (int iterate = 0; iterate < iterates; iterate++) // At most iterates iterations
    {
      Eigen::Vector2d anchor((*scenarios_)[k][0][0](0), (*scenarios_)[k][0][1](0)); // Iterations are anchored at some random constraint

      for (int obstacle_id = 0; obstacle_id < config_->max_obstacles_; obstacle_id++) // For all obstacles
      {
        double r = radii_[obstacle_id] + 1e-3; // Add some margin to the projection radius

        for (size_t s = 0; s < S; s++)
        {
          if (s == 0 && obstacle_id == 0) // The first constraint is the anchor
            continue;

          // Current constraint
          Eigen::Vector2d delta((*scenarios_)[k][obstacle_id][0](s), (*scenarios_)[k][obstacle_id][1](s));
          Eigen::Vector2d update_pose = pose;

          // Reflection Operator on the anchor
          if (std::sqrt((update_pose - anchor).transpose() * (update_pose - anchor)) < r)
            update_pose = 2.0 * (anchor - (anchor - update_pose) / (std::sqrt((update_pose - anchor).transpose() * (update_pose - anchor))) * r) - update_pose;

          // Reflection Operator on the current constraint set
          if (std::sqrt((update_pose - delta).transpose() * (update_pose - delta)) < r)
            update_pose = 2.0 * (delta - (delta - start_pose) / (std::sqrt((start_pose - delta).transpose() * (start_pose - delta))) * r) - update_pose;

          // Douglas rachford operator
          pose = (pose + update_pose) / 2.0;
        }
      }

      // STATIC CONSTRAINTS (works but may not always converge to a dynamically feasible position)
      // for (auto &halfspace : halfspaces[k].halfspaces)
      // {
      //     double r = radii_[0] + 1e-3; // Add some margin to the projection radius

      //     // Reflection Operator on the anchor
      //     Eigen::Vector2d update_pose = pose;
      //     if (std::sqrt((update_pose - anchor).transpose() * (update_pose - anchor)) < r)
      //         update_pose = 2.0 * (anchor - (anchor - update_pose) / (std::sqrt((update_pose - anchor).transpose() * (update_pose - anchor))) * r) - update_pose;

      //     // Reflection Operator on a halfspace constraint (projection/reflection onto a line)
      //     if (halfspace.A[0] * pose(0) + halfspace.A[1] * pose(1) > halfspace.b)
      //     {
      //         Eigen::Vector2d Ah(halfspace.A[0], halfspace.A[1]);
      //         update_pose = 2.0 * (update_pose + Ah * (-Ah.transpose() * update_pose - halfspace.b)) - update_pose;
      //     }

      //     // Douglas rachford operator
      //     pose = (pose + update_pose) / 2.0;
      // }
      // Stop if the change in position is close to zero
      if (RosTools::dist(prev_pose, pose) < 1e-5)
        break;

      if (iterate == iterates - 1)
      {
        ROS_ERROR("Reached max iterations in DR project.");
      }

      prev_pose = pose;
    }

    disc.SetPosition(pose);                                                           // Save the projected position in the Disc
    Eigen::Vector2d associated_vehicle_pos = disc.TranslateToVehicleCenter(plan_[k]); // Translate the disc to the vehicle position
    plan_[k].SetPosition(associated_vehicle_pos);                                     // Save the vehicle position
  }
}

void SafeHorizon::PushAlgorithm(SolverInterface *solver_interface)
{
  PROFILE_FUNCTION();

  for (size_t k = 0; k < N; k++) // For all k
  {
    auto &disc = plan_[k].discs_[disc_id_];
    Eigen::Vector2d pose = disc.AsVector2d();

    for (int obstacle_id = 0; obstacle_id < config_->max_obstacles_; obstacle_id++) // For each obstacle
    {
      Eigen::Vector2d vector_sum(0., 0.);

      // Compute a weighted direction from the scenarios, pointing outwards
      for (u_int s = 0; s < S; s += 5) // We do not really need to add every sample
      {
        Eigen::Vector2d scenario_pose((*scenarios_)[k][obstacle_id][0](s), (*scenarios_)[k][obstacle_id][1](s));
        vector_sum += pose - scenario_pose;
      }

      vector_sum.normalize();

      // Compute orthogonal vectors and check which one lines up with the vector sum better
      Eigen::Vector2d orientation_vec(std::cos(plan_[k].orientation_), std::sin(plan_[k].orientation_));
      Eigen::Vector2d orientation_orth1(-orientation_vec(1), orientation_vec(0));
      Eigen::Vector2d orientation_orth2(orientation_vec(1), -orientation_vec(0));

      double val_1 = std::abs(std::acos(vector_sum.transpose() * orientation_orth1));
      double val_2 = std::abs(std::acos(vector_sum.transpose() * orientation_orth2));
      Eigen::Vector2d orth_vec = val_1 <= val_2 ? orientation_orth1 : orientation_orth2;

      vector_sum = orth_vec; // That one is our push direction

      // Multiply this vector with the distances, i.e., V^T d
      Eigen::VectorXd distances = (((*scenarios_)[k][obstacle_id][0].array() - pose(0)).square() + ((*scenarios_)[k][obstacle_id][1].array() - pose(1)).square()).sqrt();
      Eigen::VectorXd d_vec = vector_sum(0) * ((*scenarios_)[k][obstacle_id][0].array() - pose(0)) + vector_sum(1) * ((*scenarios_)[k][obstacle_id][1].array() - pose(1));

      // Keep only distances that are smaller than r
      for (size_t s = 0; s < S; s++)
      {
        if (distances(s) >= radii_[obstacle_id])
          d_vec(s) = -20.0;
      }

      // Find the worst
      double max_d_vec = d_vec.maxCoeff();

      if (max_d_vec + radii_[obstacle_id] > 0.)
        pose += vector_sum * (radii_[obstacle_id] + max_d_vec); // Add a push to the pose of this disc

      disc.SetPosition(pose);                                                           // Save the projected position in the Disc
      Eigen::Vector2d associated_vehicle_pos = disc.TranslateToVehicleCenter(plan_[k]); // Translate the disc to the vehicle position
      plan_[k].SetPosition(associated_vehicle_pos);                                     // Save the vehicle position
    }
  }
}

void SafeHorizon::useBackupPlan(SolverInterface *solver_interface, double velocity, double deceleration)
{
  bool also_update_initial_guess = true;

  if (velocity <= 0) // Don't start with negative velocity
    velocity = 0.;

  // Save the initial state
  plan_[0].discs_[disc_id_].x = solver_interface->State().x() + velocity * std::cos(solver_interface->State().psi()) * solver_interface->DT;
  plan_[0].discs_[disc_id_].y = solver_interface->State().y() + velocity * std::sin(solver_interface->State().psi()) * solver_interface->DT;

  plan_[0].orientation_ = solver_interface->State().psi();

  if (also_update_initial_guess)
  {
    Eigen::Vector2d associated_vehicle_pos = plan_[0].discs_[disc_id_].TranslateToVehicleCenter(plan_[0]);
    plan_[0].SetPosition(associated_vehicle_pos);
  }

  // Compute a constant deceleration plan for all k
  for (size_t k = 1; k < solver_interface->FORCES_N; k++)
  {
    // Vehicle position at k - 1
    Eigen::Vector2d prev_pos = plan_[k - 1].discs_[disc_id_].TranslateToVehicleCenter(plan_[k - 1]);

    // Update the model to get to k
    Eigen::Vector2d updated_pos(prev_pos(0) + velocity * std::cos(solver_interface->State().psi()) * solver_interface->DT,
                                prev_pos(1) + velocity * std::sin(solver_interface->State().psi()) * solver_interface->DT);

    plan_[k].SetPosition(updated_pos); // Update the new pos

    velocity -= deceleration * solver_interface->DT;

    // No backwards driving
    if (velocity <= 0.)
    {
      velocity = 0.;
    }
    else if (velocity >= 3.0) // Cap maximum speed
    {
      velocity = 3.0;
    }
  }

  if (also_update_initial_guess)
    solver_interface->LoadVehiclePredictionsToInitialPlan(plan_);
}

// Check for the x's at all k if Ax = b
bool SafeHorizon::computeActiveConstraints(SolverInterface *solver_interface, SupportSubsample &active_constraints_aggregate)
{
  // We compute poses using the output, which holds the current plan of the solver
  for (size_t k = 0; k < solver_interface->FORCES_N; k++)
  {
    // Get the orientation and pose at stage k
    verify_poses_[k] = Eigen::Vector2d(solver_interface->output(k + 1, 2) + plan_[k].discs_[disc_id_].offset * std::cos(solver_interface->output(k + 1, 4)),
                                       solver_interface->output(k + 1, 3) + plan_[k].discs_[disc_id_].offset * std::sin(solver_interface->output(k + 1, 4)));
  }

  // Compute a vector of the values
  bool feasible = true;
  int infeasible_count = 0;

  for (size_t k = 0; k < N; k++)
  {
    infeasible_count = 0;

    // For each constraints in the polygon
    for (auto &constraint : polytopes_[k].polygon_out_)
    {
      // Obtain the absolute scenario index
      int index = constraint->GetHalfspaceIndex(S);

      // Evaluate the constraint (Ax - b (<= 0))
      double constraint_value = a1_[k](index) * verify_poses_[k](0) + a2_[k](index) * verify_poses_[k](1) - b_[k](index);

      if (constraint_value > 1e-3) // If it is "Infeasible"
      {
        // Mark the result as infeasible
        infeasible_count++;
        feasible = false;
        // ROS_WARN_STREAM("Infeasible Constraint (Value = " << constraint_value << ")");

        // Behavior: If they are dynamic constraints, add infeasible constraints to the support to mark them for removal
        if (constraint->type_ == ObstacleType::DYNAMIC)
          active_constraints_aggregate.Add(*constraint->scenario_);
      }
      else if (constraint->type_ == ObstacleType::DYNAMIC && constraint_value > -1e-7) // If it is "active"
      {
        // Mark the constraint as "active"
        active_constraints_aggregate.Add(*constraint->scenario_);
      }
    }
  }

  return feasible;
}

// Function is intended more as an a posteriori check of feasibility (Not used)
bool SafeHorizon::isFeasible(SolverInterface *solver_interface)
{
  for (size_t k = 0; k < N; k++)
  {
    for (int i = 0; i < std::min(24, (int)polytopes_[k].polygon_out_.size()); i++)
    {

      ScenarioConstraint *&scenario_constraint = polytopes_[k].polygon_out_[i];
      int cur_index =
          scenario_constraint->type_ == ObstacleType::DYNAMIC ? scenario_constraint->scenario_->obstacle_idx_ * S + scenario_constraint->scenario_->idx_ : scenario_constraint->scenario_->idx_;

      // Get the disc position
      double disc_x = solver_interface->Plan(k).x() + plan_[k].offsets_[disc_id_] * std::cos(solver_interface->Plan(k).psi());
      double disc_y = solver_interface->Plan(k).y() + plan_[k].offsets_[disc_id_] * std::sin(solver_interface->Plan(k).psi());
      if (a1_[k](cur_index) * disc_x + a2_[k](cur_index) * disc_y - b_[k](cur_index) > 1e-3)
      {
        std::cout << "k = " << k << ", i = " << i << std::endl;
        return false;
      }
    }
  }
  return true;
}

SupportSubsample &SafeHorizon::RemoveActiveConstraints(const SupportSubsample &active_constraints, int num_scenarios_to_remove)
{
  int removed = 0;
  for (auto &scenario : active_constraints.scenarios_)
  {

    // Skip removed scenarios
    if (removed_scenarios_.ContainsScenario(scenario))
      continue;

    if (enable_visualization_)
    {
      LMPCC_INFO_STREAM("SH-MPC: Removing scenario with index: " << scenario.idx_ << std::endl);
    }

    for (size_t k = 0; k < N; k++)
    {
      polytopes_[k].RemoveConstraint(scenario); // First marks the scenario as "Removed"
    }

    removed_scenarios_.Add(scenario); // Note: we already checked that this one is not in removed scenarios
    removed++;

    if (removed >= num_scenarios_to_remove)
      break;
  }

  // For all stages, redo the polygon construction without the removed scenarios
  for (size_t k = 0; k < N; k++)
  {
    // Save the intersections first time round
    if (old_intersects_[k].empty())
      old_intersects_[k] = polytopes_[k].intersects_out_;

    // We construct the polytopes using the ORIGINAL values (i.e., the initial disc pose and left and right vectors)
    polytopes_[k].Search(plan_[k].discs_[disc_id_].AsVector2d(), plan_[k].orientation_, x_left_[k], x_right_[k]);
  }

  return removed_scenarios_;
}

// Insert constraints A_l * x <= b_
void SafeHorizon::SetParameters(SolverInterface *solver_interface, const RealTimeData &data, int k_solver, int &param_idx)
{
  // Do k - 1 when referring to computated values IN THIS CLASS, insert them on k IN THE SolverInterface
  assert(k_solver > 0);
  int k = k_solver - 1; // Because the initial state is skipped, we need to do "-1"

  // Insert all active constraints in this stage
  for (int l = 0; l < std::min(24, (int)polytopes_[k].polygon_out_.size()); l++)
  {

    ScenarioConstraint *&scenario_constraint = polytopes_[k].polygon_out_[l];

    int cur_index = scenario_constraint->GetHalfspaceIndex(S);
    solver_interface->setParameter(k_solver, param_idx, a1_[k](cur_index));
    solver_interface->setParameter(k_solver, param_idx, a2_[k](cur_index));
    solver_interface->setParameter(k_solver, param_idx, b_[k](cur_index));
  }

  // Insert dummies on other spots (relative to vehicle position)
  for (int l = polytopes_[k].polygon_out_.size(); l < 20 + 4; l++)
  {
    solver_interface->setParameter(k_solver, param_idx, 1.0);
    solver_interface->setParameter(k_solver, param_idx, 0.0);
    solver_interface->setParameter(k_solver, param_idx, solver_interface->InitialPlan(k).x() + 100.0);
  }
}

//-------------------------------- VISUALIZATION -----------------------------------//
void SafeHorizon::Visualize()
{
  if (enable_visualization_)
  {
    PROFILE_SCOPE("Visualization");

    // Visualise scenarios
    if (config_->draw_all_scenarios_)
      visualiseScenarios();

    if (config_->draw_selected_scenarios_)
      visualiseSelectedScenarios();

    if (config_->draw_polygon_scenarios_)
      visualisePolygonScenarios();

    if (config_->draw_constraints_)
      visualisePolygons();

    publishVisuals();
  }
}

// Visualise the predictions
void SafeHorizon::visualiseScenarios()
{
  LMPCC_INFO("SH-MPC: Visualizing All Scenarios");
  bool draw_line = true;
  bool draw_circle = true;
  bool draw_points = false;

  double alpha = 1.0 / config_->sample_size_ * 2.0;

  ROSMultiplePointMarker &scenario_points = ros_markers_->getNewMultiplePointMarker("POINTS");
  scenario_points.setScale(0.05, 0.05, 0.05);
  // scenario_points.setColor(0, 0, 0, 0.8);

  RosTools::ROSPointMarker &scenario_circles = ros_markers_->getNewPointMarker("CYLINDER");
  // scenario_circles.setColor(1, 0, 0, 1.0 / config_->sample_size_ * 2.0);

  RosTools::ROSLine &selected_scenario_trajectories = ros_markers_->getNewLine();
  selected_scenario_trajectories.setScale(0.15, 0.1); // 0.15!
  // selected_scenario_trajectories.setColor(0, 0, 0, 1.0);

  Eigen::Vector3d point_prev, point_cur;
  for (int v = 0; v < config_->max_obstacles_; v++) // For all obstacles
  {
    for (size_t s = 0; s < S; s++) // For all samples
    {
      for (uint k = 0; k < config_->indices_to_draw_.size(); k++) // For all drawn indices
      {
        int index = config_->indices_to_draw_[k];
        point_cur = getScenarioLocation(index, v, s);
        point_cur(2) = -0.1;

        if (draw_points)
        {
          scenario_points.setColorInt(k, config_->indices_to_draw_.size(), 1.0);
          scenario_points.addPointMarker(point_cur); // Add a point at the scenario
        }
        if (draw_circle)
        {
          scenario_circles.setColorInt(k, config_->indices_to_draw_.size(), alpha);
          scenario_circles.setScale(2 * (radii_[v] - config_->vehicle_width_ / 2.), 2 * (radii_[v] - config_->vehicle_width_ / 2.), 0.01);
          scenario_circles.addPointMarker(point_cur); // Add a circle with the collision radius
        }
        if (draw_line && k != 0)
        {
          selected_scenario_trajectories.setColorInt(k, config_->indices_to_draw_.size(), 1.0); // 0.2); // 1.0);
          selected_scenario_trajectories.addLine(point_prev, point_cur);                        // Add a line
        }
        point_prev = point_cur;
      }
    }
  }

  if (draw_points)
    scenario_points.finishPoints();
}

void SafeHorizon::visualiseSelectedScenarios()
{
  bool draw_circles = true;

  LMPCC_INFO("SH-MPC: Visualizing Selected Scenarios");

  RosTools::ROSPointMarker &scenario_points = ros_markers_->getNewPointMarker("CYLINDER");
  scenario_points.setScale(0.15, 0.15, 0.1e-3);

  RosTools::ROSPointMarker &scenario_circles = ros_markers_->getNewPointMarker("CYLINDER");

  RosTools::ROSLine &selected_scenario_trajectories = ros_markers_->getNewLine();
  selected_scenario_trajectories.setScale(0.07 * config_->scenario_visual_scale_, 0.07 * config_->scenario_visual_scale_);

  Eigen::Vector3d scenario_location, prev_location;

  // Plot support scenarios (and removed scenarios with a different color)
  // For all scenarios of support -> Plots support subsample, which includes removed scenarios!
  for (Scenario &scenario : support_subsample_.scenarios_)
  {
    bool is_removed = removed_scenarios_.ContainsScenario(scenario);
    if (is_removed && !config_->draw_removed_scenarios_)
      continue;

    // To plot the whole obstacle, we need to find all the indices that belong to the same obstacle
    DynamicObstacle *scenario_obstacle = nullptr;

    int disc_id = 0;
    int obs_id = 0;

    // For each obstacle
    for (auto &obstacle : *obstacles_)
    {
      int temp_obs_id = obs_id; // This is the index starting with this obstacle
      for (size_t d = 0; d < obstacle.discs_.size(); d++)
      {
        if (scenario.obstacle_idx_ == disc_id)
        {
          scenario_obstacle = &obstacle;
        }
        disc_id++;
        temp_obs_id++;
      }

      if (scenario_obstacle)
        break;

      obs_id = temp_obs_id; // If it wasn't our obstacle, keep counting onwards
    }

    for (size_t k = 0; k < config_->indices_to_draw_.size(); k++)
    {
      const int &index = config_->indices_to_draw_[k];

      if (is_removed)
        selected_scenario_trajectories.setColor(1.0, 0.0, 0.0, 0.8);
      else
        selected_scenario_trajectories.setColorInt(k, (int)config_->indices_to_draw_.size());

      // Retrieve the scenario location
      scenario_location = getScenarioLocation(index, scenario.obstacle_idx_, scenario.idx_);
      scenario_location(2) = -((double)k) * 0.1e-3;

      // Draw a broken line
      if (k > 0)
      {
        // selected_scenario_trajectories.addBrokenLine(prev_location, scenario_location, 0.2);
        selected_scenario_trajectories.addLine(prev_location, scenario_location);
      }
      prev_location = scenario_location;

      // Draw a circle for each disc of the obstacle where the scenario is of support
      for (auto &disc : scenario_obstacle->discs_)
      {
        int idx = disc.id;
        scenario_location = getScenarioLocation(index, idx, scenario.idx_);
        scenario_location(2) = -((double)k) * 0.1e-3;

        scenario_circles.setScale(2 * (radii_[scenario.obstacle_idx_] - config_->vehicle_width_ / 2.), 2 * (radii_[scenario.obstacle_idx_] - config_->vehicle_width_ / 2.), 0.1);

        if (draw_circles)
        {
          scenario_circles.setColorInt(k, config_->indices_to_draw_.size(), 0.4);
          scenario_points.setColorInt(k, config_->indices_to_draw_.size(), 1.0);

          scenario_circles.addPointMarker(scenario_location);
          scenario_points.addPointMarker(scenario_location);
        }
      }
    }
  }
}

// Visualizes scenarios that make up the polygon, not necessarily of support
void SafeHorizon::visualisePolygonScenarios()
{
  LMPCC_INFO("SH-MPC: Visualizing Polygon Scenarios");

  bool plot_line = true;
  bool plot_positions = false;

  std::vector<ROSMultiplePointMarker *> polygon_positions; // Vector necessary to maintain coloring per k

  for (size_t k = 0; k < config_->indices_to_draw_.size(); k++)
  {
    polygon_positions.push_back(&ros_markers_->getNewMultiplePointMarker("SPHERE"));
    polygon_positions[k]->setScale(0.1, 0.1, 0.01);
  }

  RosTools::ROSLine &polygon_scenario_trajectories = ros_markers_->getNewLine();
  polygon_scenario_trajectories.setScale(0.05 * config_->scenario_visual_scale_, 0.1);

  Eigen::Vector3d scenario_location, prev_location;

  SupportSubsample polygon_constraints;

  // Aggregate constraints along the horizon
  for (size_t k = 0; k < N; k++)
  {
    // Get the polygon for this stage
    std::vector<ScenarioConstraint *> &constraints_ = polytopes_[k].polygon_out_;

    for (size_t i = 0; i < constraints_.size(); i++)
    {
      if (constraints_[i]->type_ == ObstacleType::DYNAMIC)
        polygon_constraints.Add(*constraints_[i]->scenario_);
    }
  }

  // For all scenarios of support -> Plots support subsample, which includes removed scenarios!
  for (int i = 0; i < polygon_constraints.support_subsample_size_; i++)
  {

    for (size_t k = 0; k < config_->indices_to_draw_.size(); k++)
    {

      const int &index = config_->indices_to_draw_[k];

      scenario_location = getScenarioLocation(index, polygon_constraints.scenarios_[i].obstacle_idx_, polygon_constraints.scenarios_[i].idx_);
      scenario_location(2) = -((double)k) * 0.1;

      // Draw the point
      if (plot_positions)
      {
        polygon_positions[k]->setColorInt(k, (int)config_->indices_to_draw_.size(), 0.5);
        polygon_positions[k]->addPointMarker(scenario_location);
      }

      // Draw a line
      if (k > 0 && plot_line)
      {
        polygon_scenario_trajectories.setColorInt(k, (int)config_->indices_to_draw_.size(), 0.3);
        polygon_scenario_trajectories.addBrokenLine(prev_location, scenario_location, 0.2);
      }

      prev_location = scenario_location;
    }
  }

  for (auto &marker : polygon_positions)
    marker->finishPoints();
}

void SafeHorizon::visualisePolygons()
{
  LMPCC_INFO("SH-MPC: Visualizing Polygons");

  RosTools::ROSLine &line = ros_markers_->getNewLine();
  line.setScale(0.1 * config_->scenario_visual_scale_, 0.1 * config_->scenario_visual_scale_);

  geometry_msgs::Point p1, p2;
  p1.z = 0.3e-3;
  p2.z = 0.3e-3;

  bool visualize_points = false;
  RosTools::ROSPointMarker &intersections = ros_markers_->getNewPointMarker("CYLINDER");
  intersections.setScale(0.1 * config_->scenario_visual_scale_, 0.1 * config_->scenario_visual_scale_, 1e-3);

  for (size_t k = 0; k < config_->indices_to_draw_.size(); k++)
  {

    int &index = config_->indices_to_draw_[k];
    line.setColorInt(k, (int)config_->indices_to_draw_.size());
    intersections.setColorInt(k, (int)config_->indices_to_draw_.size());

    p1.z = ((double)k) * 0.2e-2; // Above all other scenario visuals
    p2.z = ((double)k) * 0.2e-2;

    std::vector<Eigen::Vector2d *> &intersects = polytopes_[index].intersects_out_;

    for (size_t i = 0; i < intersects.size(); i++)
    {

      // Draw lines between consecutive intersections
      if (i == 0)
      {
        p1.x = (*intersects[intersects.size() - 1])(0);
        p1.y = (*intersects[intersects.size() - 1])(1);
      }
      else
      {
        p1.x = (*intersects[i - 1])(0);
        p1.y = (*intersects[i - 1])(1);
      }

      p2.x = (*intersects[i])(0);
      p2.y = (*intersects[i])(1);

      line.addLine(p1, p2);

      if (visualize_points) // If true, draw the corners as well
      {
        intersections.addPointMarker(p1);
      }
    }
  }

  // visualizeAllConstraints();
}

// Debug functionality
void SafeHorizon::visualizeAllConstraints()
{
  LMPCC_INFO("SH-MPC: Visualizing All Constraints");

  RosTools::ROSLine &line = ros_markers_->getNewLine();
  geometry_msgs::Point p1, p2;
  p1.z = 0.5;
  p2.z = 0.5; // 2D points

  int k = 19; // draw only for this k
  bool plot_all = false;
  bool plot_top_bottom = true;
  bool plot_intersects = true;

  if (plot_all)
  {
    for (size_t i = 0; i < constraints_[k].size(); i++)
    {

      p1.x = x_left_[k];
      p1.y = y_left_[k](i);

      p2.x = x_right_[k];
      p2.y = y_right_[k](i);

      line.addLine(p1, p2);
    }
  }

  if (plot_top_bottom)
  {
    line.setColor((double)1.0 / 3.0 / 4.0);
    for (size_t i = 0; i < polytopes_[k].result_indices_bot_.size(); i++)
    {

      // Index of a selected line
      int index = polytopes_[k].result_indices_bot_[i];

      p1.x = x_left_[k];
      p1.y = y_left_[k](index);

      p2.x = x_right_[k];
      p2.y = y_right_[k](index);

      line.addLine(p1, p2);
    }

    line.setColor((double)2.0 / 3.0 / 4.0);

    for (size_t i = 0; i < polytopes_[k].result_indices_top_.size(); i++)
    {

      // Index of a selected line
      int index = polytopes_[k].result_indices_top_[i];

      p1.x = x_left_[k];
      p1.y = y_left_[k](index);

      p2.x = x_right_[k];
      p2.y = y_right_[k](index);

      line.addLine(p1, p2);
    }
  }

  if (plot_intersects)
  {
    RosTools::ROSPointMarker &intersections = ros_markers_->getNewPointMarker("CUBE");
    intersections.setScale(0.4, 0.4, 0.1);
    intersections.setColor(1, 0, 0);
    for (size_t i = 0; i < polytopes_[k].result_intersects_bot_.size(); i++)
    {

      // Index of a selected line
      p1.x = polytopes_[k].result_intersects_bot_[i](0);
      p1.y = polytopes_[k].result_intersects_bot_[i](1);

      intersections.addPointMarker(p1);
    }
    for (size_t i = 0; i < polytopes_[k].result_intersects_top_.size(); i++)
    {

      // Index of a selected line
      p1.x = polytopes_[k].result_intersects_top_[i](0);
      p1.y = polytopes_[k].result_intersects_top_[i](1);

      intersections.addPointMarker(p1);
    }
  }
}

Eigen::Vector3d SafeHorizon::getScenarioLocation(const int &k, const int &obstacle_index, const int &scenario_index)
{

  Eigen::Vector2d loc((*scenarios_)[k][obstacle_index][0](scenario_index), (*scenarios_)[k][obstacle_index][1](scenario_index));
  return Eigen::Vector3d(loc(0), loc(1), 0.2);
}

Eigen::Vector2d SafeHorizon::getScenarioLocation2D(const int &k, const int &obstacle_index, const int &scenario_index)
{
  return Eigen::Vector2d((*scenarios_)[k][obstacle_index][0](scenario_index), (*scenarios_)[k][obstacle_index][1](scenario_index));
}

void SafeHorizon::publishVisuals()
{
  // Draws all the markers added to ros_markers_ and resets
  ros_markers_->publish();
}