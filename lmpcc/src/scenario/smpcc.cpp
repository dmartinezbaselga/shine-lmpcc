#include "scenario/smpcc.h"
#include "lmpcc_solver/SolverInclude.h"

SMPCC::SMPCC(ros::NodeHandle &nh, predictive_configuration *config, const int disc_id, GaussianSampler &sampler, bool enable_visualization = false)
    : config_(config), enable_visualization_(enable_visualization), disc_id_(disc_id)
{
  LMPCC_WARN_ALWAYS("Initializing Scenario Manager");
  // Initialise the visualisation
  ros_markers_.reset(new RosTools::ROSMarkerPublisher(nh, "scenario_constraints/markers", config_->target_frame_, 1800));

  sampler_ = &sampler;

  // Save some useful variables
  if (config->use_real_samples_)
    S = config->sample_size_;
  else
    S = sampler_->SampleSize(); // config_->sample_size_;

  N = predictive_configuration::N;

  R_ = config_->removal_count_;
  l_ = config_->polygon_checked_constraints_;

  // Resize over the horizon
  poses_.resize(N);
  orientations_.resize(N);
  projected_poses_.resize(N);
  scenario_threads_.resize(N);

  int constraint_size = config_->max_obstacles_ * S + config_->n_halfspaces_;

  // Indices used for finding the closest scenarios to remove
  sort_indices_.resize(S * config_->max_obstacles_);

  // Initialize polygon constructors
  for (u_int k = 0; k < N; k++)
    local_polygons_.emplace_back(constraint_size, config_->polygon_range_);

  active_obstacle_count_.resize(N, 0);
  active_obstacle_indices_.resize(N);
  for (uint k = 0; k < N; k++)
    active_obstacle_indices_[k].resize(config_->max_obstacles_);

  areas_.resize(N);
  for (u_int k = 0; k < N; k++)
    areas_[k] = 0.0;

  radii_.resize(config_->max_obstacles_);

  LMPCC_SUCCESS_ALWAYS("Scenario-MPCC: Initialized");
}

void SMPCC::Update(SolverInterface *solver_interface, RealTimeData &data)
{
  LMPCC_INFO("SMPCC::Update");

  obstacles_ = (std::vector<DynamicObstacle> *)(&data.dynamic_obstacles_);

  copied_obstacles_ = data.dynamic_obstacles_; // Thread safe

  // No objects or predictions (if used) received
  if (copied_obstacles_.size() == 0 || (!config_->use_real_samples_ && copied_obstacles_[0].prediction_.gaussians[0].mean.poses.size() == 0))
  {
    if (config_->debug_output_)
      ROS_WARN("Stage Scenario: No obstacles received yet. Update skipped!");

    return;
  }

  status_ = ScenarioStatus::SUCCESS;

  // Translate Gaussian samples to the observations and save the pointer
  LMPCC_INFO("SMPCC: Preparing Samples");

  // if (config_->use_json_trajectory && config_->use_real_samples_)
  // {

  // Sampler::Get().SampleJsonRealTrajectories(data.dynamic_obstacles_, partitioning_);
  //     scenarios_ = Sampler::Get().SamplePredictions();
  // }

  // else if (config_->use_real_samples_)
  // {
  //     scenarios_ = Sampler::Get().SampleRealTrajectories(data.dynamic_obstacles_, partitioning_);
  // }
  {
    PROFILE_SCOPE("Preparing Samples");
    if (config_->use_real_samples_)
    {
      // Sampler::Get().SampleJsonRealTrajectories(data); // Should happen once! For now fine

      scenarios_ = Sampler::Get().SamplePredictions();
    }
    else
      scenarios_ = sampler_->TranslateToMeanAndVariance(copied_obstacles_);
  }
  LMPCC_INFO("SMPCC: Samples Ready");

  // Compute the poses and orientation for this disc
  SCENARIO_INFO("Retrieving Vehicle Trajectory");

  plan_ = solver_interface->InitialVehiclePrediction();

  // Get the radii for each of the obstacles
  int disc_id = 0;
  for (size_t v = 0; v < copied_obstacles_.size(); v++)
  {
    for (size_t d = 0; d < copied_obstacles_[v].discs_.size(); d++)
    {
      radii_[disc_id] = copied_obstacles_[v].discs_[d].radius + config_->vehicle_width_ / 2.;
      disc_id++;
    }
  }

  PushAlgorithm(solver_interface, data);

  LMPCC_INFO("SMPCC: Constructing Constraints From Scenarios");

#pragma omp parallel for num_threads(8)
  for (size_t k = 0; k < N; k++)
  {
    scenariosToConstraints(k, data.halfspaces_[k]);
  }
  // }

  LMPCC_INFO("SMPCC: Done");
}

void SMPCC::Visualize()
{

  PROFILE_FUNCTION();

  if (enable_visualization_)
  {
    // Visualise scenarios
    if (config_->draw_all_scenarios_)
      visualiseScenarios();

    if (config_->draw_polygon_scenarios_)
      visualisePolygonScenarios();

    // if (config_->draw_removed_scenarios_)
    //     visualiseRemovedScenarios(config_->indices_to_draw_);

    if (config_->draw_constraints_)
    {
      // Draw the polygons using the lines from the constructors
      // visualisePolygonScenarios();

      visualisePolygons();
    }

    publishVisuals();
  }
}

// Constructs N x m linear constraints for all stages and the given scenarios_ (result = scenario_constraints_)
void SMPCC::scenariosToConstraints(int k, const std::vector<RosTools::Halfspace> &halfspaces)
{
  PROFILE_FUNCTION();
  // Reset the polygon class
  local_polygons_[k].Reset(); // Not necessary

  // Make a local copy that changes
  // Eigen::Vector2d pose = disc_->poses_[k];
  Eigen::Vector2d pose = plan_[k].discs_[disc_id_].AsVector2d();

  //------------------- Verify the relevance of obstacles ----------------------------------//
  // active_obstacle_count_[k] = 0;

  // Check all obstacles
  // for (uint v = 0; v < (unsigned int)config_->max_obstacles_; v++)
  // {
  //     auto &obstacle = (*obstacles_)[v];

  //     // If the vehicle is near one of the means
  //     for (auto &gaussian : obstacle.prediction_.gaussians)
  //     {
  //         Eigen::Vector2d obst(gaussian.mean.poses[k].pose.position.x,
  //                              gaussian.mean.poses[k].pose.position.y);
  //         // If this obstacle is relevant
  //         if (RosTools::dist(obst, pose) <= config_->activation_range_)
  //         {
  //             // Save that this obstacle is active for this stage, increase the count
  //             active_obstacle_indices_[k][active_obstacle_count_[k]] = v;
  //             active_obstacle_count_[k]++;
  //         }

  //         break;
  //     }
  // }
  //

  // SCENARIO_INFO("Computing Distances");

  //--------------------- Construct constraints for all non-removed points ----------------------//
  {
    // PROFILE_SCOPE("Constructing Halfspaces");

    // Here we compute the constraints for all obstacles
    LocalPolygon::PointArray point_array;
    for (int v = 0; v < config_->max_obstacles_; v++)
    {
      // std::cout << v << std::endl;
      // Retrieve the points
      point_array.x_ = &((*scenarios_)[k][v][0]);
      point_array.y_ = &((*scenarios_)[k][v][1]); // Can be improved by using PointArray in sampler
      point_array.r_ = radii_[v];

      // Convert the points to halfspaces
      local_polygons_[k].constraints_.PointArrayToHalfspaces(point_array, pose, v);
    }

    //
    // Sort on distance and remove the closest scenarios
    /*if (R_ > 0)
        removeScenariosBySorting(k);*/

    // SCENARIO_INFO("Stage Scenario: Finalizing Constraints");

    // To adapt to the polygon data structure
    LocalPolygon::HalfspaceArray external_halfspaces(config_->n_halfspaces_);
    for (int i = 0; i < config_->n_halfspaces_; i++)
    {
      external_halfspaces.a1_(i) = halfspaces[i].A_[0];
      external_halfspaces.a2_(i) = halfspaces[i].A_[1];
      external_halfspaces.b_(i) = halfspaces[i].b_;
    }

    local_polygons_[k].constraints_.AddHalfspaces(external_halfspaces, true);
  }
  // DEBUG: Print constraints for one stage
  // if (k == 2)
  // local_polygons_[k].constraints_.PrintConstraints();
  //

  // SCENARIO_INFO("Stage Scenario: Constructing Polygons");

  {
    // PROFILE_SCOPE("Polygon Search");

    // local_polygons_[k].constraints_.PrintConstraints();
    local_polygons_[k].Search(pose, orientations_[k]);
    // polygon_out_ has the scenario constraints
  }

  areas_[k] = 0;
}

void SMPCC::removeScenariosBySorting(int k)
{

  std::iota(sort_indices_.begin(), sort_indices_.end(), 0);

  const Eigen::ArrayXd &distances = local_polygons_[k].constraints_.GetDistances();
  // Removal based on distance to VRU mean!
  std::sort(sort_indices_.begin(), sort_indices_.end(), [&](const int &a, const int &b)
            { return distances(a) < distances(b); });

  // Remove the first R_ constraints
  const std::vector<LocalPolygon::ConstraintInfo> &constraint_info = local_polygons_[k].constraints_.GetConstraintInfoRef();
  for (int i = 0; i < R_; i++)
    local_polygons_[k].RemoveConstraint(constraint_info[sort_indices_[i]].id_); // scenario_indices_[k][sort_indices_[i]]);
}

void SMPCC::feasibilityCheck(int k)
{
  // Eigen::Vector2d pose = plan_[k].discs_[disc_id_].AsVector2d();

  // Eigen::ArrayXd constraint_values = a1_[k] * pose(0) + a2_[k] * pose(1) - b_[k];

  // for (u_int s = 0; s < S; s++)
  // {
  //     if (constraint_values(s) > 1e-3)
  //     {
  //         is_feasible_ = false;
  //         return;
  //     }
  // }
}

void SMPCC::PushAlgorithm(SolverInterface *solver_interface, const RealTimeData &data)
{
  PROFILE_FUNCTION();
  for (size_t k = 0; k < N; k++)
  {
    auto &disc = plan_[k].discs_[disc_id_];
    Eigen::Vector2d pose = disc.AsVector2d();

    // Push from static obstacles
    for (int i = 0; i < config_->n_halfspaces_; i++)
    {

      const auto &halfspace = data.halfspaces_[k][i];

      if (halfspace.A_.transpose() * pose - halfspace.b_ > 1e-8)                 // If violated
        pose += halfspace.A_ * (halfspace.A_.transpose() * pose - halfspace.b_); // Project to a safe position from the line
    }

    for (int obstacle_id = 0; obstacle_id < config_->max_obstacles_; obstacle_id++)
    {
      Eigen::Vector2d vector_sum(0., 0.);

      for (u_int s = 0; s < S; s += 5) // We do not really need to add every sample
      {
        Eigen::Vector2d scenario_pose((*scenarios_)[k][obstacle_id][0](s), (*scenarios_)[k][obstacle_id][1](s));
        vector_sum += pose - scenario_pose;
      }
      vector_sum.normalize();

      double biggest_push = -radii_[obstacle_id];
      // V^T d
      Eigen::VectorXd d_vec = vector_sum(0) * ((*scenarios_)[k][obstacle_id][0].array() - pose(0)) + vector_sum(1) * ((*scenarios_)[k][obstacle_id][1].array() - pose(1));

      // Find the worst
      double max_d_vec = d_vec.maxCoeff();
      if (max_d_vec > biggest_push)
        biggest_push = max_d_vec;

      pose += vector_sum * (biggest_push + radii_[obstacle_id]); // Add a push to the pose of this disc
    }

    // pose += vector_sum * (biggest_push + config_->ego_w_ / 2.); // Add a push

    // disc_->poses_[k] = pose; // Set the new disc pose
    disc.SetPosition(pose);                                                           // Save the projected position in the Disc
    Eigen::Vector2d associated_vehicle_pos = disc.TranslateToVehicleCenter(plan_[k]); // Translate the disc to the vehicle position
    plan_[k].SetPosition(associated_vehicle_pos);                                     // Save the vehicle position
                                                                                      // Eigen::Vector2d veh_pose = disc_->DiscPoseToVehiclePoseAt(k); // Translate to the vehicle pose
                                                                                      // solver_interface->InitialPlan(k).set_x(veh_pose(0));
                                                                                      // solver_interface->InitialPlan(k).set_y(veh_pose(1));
  }
}

void SMPCC::useBackupPlan(SolverInterface *solver_interface, double vel, double deceleration)
{
  LMPCC_INFO("Stage Scenario: Finding a backup plan");

  // First, try to find out what deceleration is sufficient (For the front disc!)
  // double v = 0.; // solver_interface->State().get_v();
  double angle = solver_interface->State().psi();

  Disc &disc = plan_[0].discs_[disc_id_];

  // // Current disc position
  Eigen::Vector2d cur_pose(solver_interface->State().x(), solver_interface->State().y());
  cur_pose += Eigen::Vector2d(std::cos(angle) * disc.offset, std::sin(angle) * disc.offset); // Account for the disc offset

  // // Function to compute vehicle pose under constant velocity
  auto get_vehicle_pose = [&](const Eigen::Vector2d &start_pose, int T, double velocity, Eigen::Vector2d &pose_out)
  { pose_out = (start_pose + Eigen::Vector2d(std::cos(angle), std::sin(angle)) * (velocity * ((double)(T)) * solver_interface->DT)); };

  // Is still questionable
  solver_interface->InitialPlan(0) = solver_interface->State(); // Set the initial state to match the current measurements

  // More involved: compute deceleration for safety
  // double deceleration = getSafeDeceleration(solver_interface);
  // Simple: specified value
  // double deceleration = config_->deceleration_at_infeasible_;

  // double angle = solver_interface->psi(0);
  // Is it correct that cur_pose is the current state?
  Eigen::Vector2d pose_k, prev_pose;
  for (size_t k = 0; k < N; k++)
  {
    double velocity = /*solver_interface->v()*/ vel - deceleration * solver_interface->DT * k; // v -= a*T, T = k*dt
    if (velocity < 0.)                                                                         // No rear driving
      velocity = 0;

    if (k == 0)
    {
      pose_k = cur_pose;
    }
    else
    {

      solver_interface->v(k) = velocity;

      // Set the solver values (Note: reverts back to disc positions)
      get_vehicle_pose(prev_pose, 1, velocity, pose_k);
      auto &state_k = solver_interface->InitialPlan(k);
      state_k.set_x(pose_k(0));
      state_k.set_y(pose_k(1));
      state_k.set_psi(pose_k(angle));
    }
    prev_pose = pose_k;
  }
}

// Insert chance constraints into the optimisation
void SMPCC::SetParameters(SolverInterface *solver_interface, const RealTimeData &data, int k, int &param_idx)
{
  // Insert constraints A_l * x <= b_l
  assert(k > 0);

  int k_planning = k - 1;

  if (k == 0)
    LMPCC_INFO("SMPCC::SetParameters");

  // Insert all active constraints in this stage
  for (int l = 0; l < std::min(24, (int)local_polygons_[k_planning].output_.NumConstraints()); l++)
  {
    LocalPolygon::Halfspace &halfspace = local_polygons_[k_planning].output_.constraints_[l];

    solver_interface->setParameter(k, param_idx, halfspace.A_(0, 0));
    solver_interface->setParameter(k, param_idx, halfspace.A_(0, 1));
    solver_interface->setParameter(k, param_idx, halfspace.b_);
  }

  // Insert dummies on other spots (relative to vehicle position)
  for (int l = std::min(24, (int)local_polygons_[k_planning].output_.NumConstraints()); l < 20 + 4; l++)
  {
    solver_interface->setParameter(k, param_idx, 1.0);
    solver_interface->setParameter(k, param_idx, 0.0);
    solver_interface->setParameter(k, param_idx, solver_interface->x(k_planning) + 100.0);
  }
}

//-------------------------------- VISUALIZATION -----------------------------------//
// Visualise the predictions
void SMPCC::visualiseScenarios()
{
  if (obstacles_->size() <= 0)
    return;

  Eigen::Vector3d point_cur;

  // #pragma omp parallel for
  for (size_t k = 0; k < config_->indices_to_draw_.size(); k++) // Draw per k to fix the colors
  {
    int index = config_->indices_to_draw_[k];

    ROSMultiplePointMarker &scenario_points = ros_markers_->getNewMultiplePointMarker("POINTS"); // We generate multiple point marker for each k, to fix the color
    scenario_points.setScale(0.1, 0.1, 0.1);
    scenario_points.setColorInt(k, (int)config_->indices_to_draw_.size(), 0.4);

    for (int v = 0; v < config_->max_obstacles_; v++)
    {
      for (size_t s = 0; s < S; s++)
      {

        point_cur = getScenarioLocation(index, Scenario{(int)s, v});
        point_cur(2) = 0.05;

        scenario_points.addPointMarker(point_cur);
      }
    }
    scenario_points.finishPoints();
  }
}

int SMPCC::constraintIndexToScenarioIndex(int constraint_index) { return constraint_index % S; }

void SMPCC::visualiseRemovedScenarios(const std::vector<int> &indices_to_draw)
{

  // ROSMultiplePointMarker &removed_scenarios_ = ros_markers_->getNewMultiplePointMarker("POINTS");
  // removed_scenarios_.setScale(0.15, 0.15, 0.15);
  // removed_scenarios_.setColor(0, 0.7, 0);

  // for (size_t k = 0; k < indices_to_draw.size(); k++)
  // {
  //     const int &index = indices_to_draw[k];

  //     // Draw removed and selected scenarios
  //     for (size_t i = 0; i < S; i++)
  //     {
  //         for (int v = 0; v < active_obstacle_count_[index]; v++)
  //         {
  //             // Get the index of this obstacle
  //             int obst = active_obstacle_indices_[index][v];

  //             // Plot removed constraints
  //             if (!local_polygons_[index].WasRemoved(scenario_indices_[k][obst * S + i]))
  //                 continue;

  //             Eigen::Vector3d scenario_location = getScenarioLocation(index, obst, i);

  //             // Draw a point
  //             removed_scenarios_.addPointMarker(scenario_location);
  //         }
  //     }
  // }

  // removed_scenarios_.finishPoints();
}

void SMPCC::visualiseSelectedScenarios(const std::vector<int> &indices_to_draw)
{

  // RosTools::ROSPointMarker &selected_scenario_points = ros_markers_->getNewPointMarker("CUBE");
  // selected_scenario_points.setScale(0.1, 0.1, 0.1);
  // selected_scenario_points.setColor(1, 1, 0);

  // RosTools::ROSPointMarker &selected_scenario_circles = ros_markers_->getNewPointMarker("CYLINDER"); // Just the obstacles themselves
  // selected_scenario_circles.setScale(2 * (config_->r_VRU_), 2 * (config_->r_VRU_), 0.01);
  // selected_scenario_circles.setColor(1, 1, 0, 0.025);
  // RosTools::ROSPointMarker &selected_full_circles = ros_markers_->getNewPointMarker("CYLINDER"); // Includes the vehicle!
  // selected_scenario_circles.setColor(1, 1, 0, 0.025);

  // for (uint k = 0; k < indices_to_draw.size(); k++)
  // {
  //     const int &index = indices_to_draw[k];
  //     selected_scenario_points.setColor((double)k / (double)indices_to_draw.size() / 4.0);
  //     selected_scenario_circles.setColor((double)k / (double)indices_to_draw.size() / 4.0, 0.1);

  //     // Get lines in the polygon
  //     std::vector<ScenarioConstraint *> &constraints = polygon_constructors_[index].polygon_out_;

  //     for (size_t i = 0; i < constraints.size(); i++)
  //     {

  //         if (constraints[i]->type_ != ObstacleType::DYNAMIC)
  //             continue;

  //         Eigen::Vector3d scenario_location = getScenarioLocation(
  //             index,
  //             constraints[i]->scenario_->obstacle_idx_,
  //             constraints[i]->scenario_->idx_);

  //         // Draw a yellow point
  //         scenario_location(2) = 0.5;
  //         selected_scenario_points.addPointMarker(scenario_location);

  //         // Draw a circle around it
  //         if (config_->r_VRU_ > 0.0)
  //         {
  //             selected_scenario_circles.addPointMarker(scenario_location);
  //         }
  //         selected_full_circles.addPointMarker(scenario_location);
  //     }
  // }
}

void SMPCC::visualisePolygonScenarios()
{
  LMPCC_INFO_FUNCTION;

  RosTools::ROSPointMarker &polygon_scenario_points = ros_markers_->getNewPointMarker("CUBE");
  polygon_scenario_points.setScale(0.15, 0.15, 0.1);
  polygon_scenario_points.setColor(1, 1, 0, 0.6);

  RosTools::ROSPointMarker &selected_scenario_circles = ros_markers_->getNewPointMarker("CYLINDER");
  selected_scenario_circles.setScale(2 * (config_->r_VRU_), 2 * (config_->r_VRU_), 0.01); // + config_->ego_w_ / 2.0
  selected_scenario_circles.setColor(1, 1, 0, 0.02);

  // RosTools::ROSPointMarker &selected_full_circles = ros_markers_->getNewPointMarker("CYLINDER"); // Includes the vehicle!
  // selected_full_circles.setScale(2 * (radii_[0]), 2 * (radii_[0]), 0.01);
  // selected_full_circles.setColor(1, 1, 0, 0.01);
  Eigen::Vector3d scenario_location;

  std::vector<SupportSubsample> polygon_constraints;

  // Aggregate constraints along the horizon
  for (size_t k = 0; k < config_->indices_to_draw_.size(); k++)
  {
    const int &index = config_->indices_to_draw_[k];
    polygon_constraints.emplace_back(50);

    // Get the polygon for this stage
    std::vector<LocalPolygon::Halfspace> &constraints = local_polygons_[index].output_.constraints_;

    for (size_t i = 0; i < constraints.size(); i++)
    {
      // Add the scenario if it is dynamic and not yet contained in the set
      if (constraints[i].info_->type_ == LocalPolygon::ObstacleType::DYNAMIC)
      {
        // Map the constraint index to that of the scenario for identification!
        int scenario_id = constraintIndexToScenarioIndex(constraints[i].info_->id_);

        polygon_constraints[k].Add({scenario_id, constraints[i].info_->obstacle_id_}); // Checks for duplicates internally
      }
    }
  }

  // For all scenarios of support -> Plots support subsample, which includes removed scenarios!
  for (size_t k = 0; k < config_->indices_to_draw_.size(); k++)
  {
    const int &index = config_->indices_to_draw_[k];

    for (int i = 0; i < polygon_constraints[k].support_subsample_size_; i++)
    {
      // Draw each constraints along the horizon
      Scenario &scenario = polygon_constraints[k].scenarios_[i];

      polygon_scenario_points.setColorInt(k, (int)config_->indices_to_draw_.size(), 1.0);
      selected_scenario_circles.setColorInt(k, (int)config_->indices_to_draw_.size(), 0.15);

      // Get the location of this scenario
      scenario_location = getScenarioLocation(index, scenario);

      scenario_location(2) = k * 0.1;
      polygon_scenario_points.addPointMarker(scenario_location);

      // Draw a circle around it
      if (config_->r_VRU_ > 0.0)
        selected_scenario_circles.addPointMarker(scenario_location);

      // selected_full_circles.addPointMarker(scenario_location);
    }
  }
}

void SMPCC::visualisePolygons()
{
  LMPCC_INFO_FUNCTION;
  RosTools::ROSLine &line = ros_markers_->getNewLine();
  geometry_msgs::Point p1, p2;
  p1.z = 0.3;
  p2.z = 0.3; // 2D points
  // RosTools::ROSPointMarker &intersections = ros_markers_->getNewPointMarker("CUBE");
  // intersections.setScale(0.25, 0.25, 0.1);
  // intersections.setColor(0, 0, 0);

  for (size_t k = 0; k < config_->indices_to_draw_.size(); k++)
  {

    int &index = config_->indices_to_draw_[k];
    line.setColorInt(k, (int)config_->indices_to_draw_.size());
    line.setScale(0.1, 0.1);
    p1.z = 0.3;
    p2.z = 0.3;
    std::vector<Eigen::Vector2d *> &intersects = local_polygons_[index].output_.intersections_;

    for (size_t i = 0; i < intersects.size(); i++)
    {
      // Consecutive intersections
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
      // // std::cout << "Drawing line from " << p1.x << ", " << p1.y << " to " << p2.x << ", " << p2.y << std::endl;

      line.addLine(p1, p2);

      // intersections.addPointMarker(p1);
    }
  }
}

void SMPCC::visualiseProjectedPosition()
{

  RosTools::ROSPointMarker &selected_scenario_circles = ros_markers_->getNewPointMarker("CYLINDER");
  selected_scenario_circles.setScale(1.0, 1.0, 0.01); // + config_->ego_w_ / 2.0
  selected_scenario_circles.setColor(1, 1, 1, 0.7);

  for (uint k = 0; k < config_->indices_to_draw_.size(); k++)
  {
    const int &index = config_->indices_to_draw_[k];

    Eigen::Vector3d scenario_location(projected_poses_[index](0), projected_poses_[index](1), 0.2);

    selected_scenario_circles.addPointMarker(scenario_location);
  }
}

Eigen::Vector3d SMPCC::getScenarioLocation(const int &k, const int &s, const int &v)
{
  assert(false);
  return Eigen::Vector3d();
  // return getScenarioLocation(k, scenario_info_[k][scenarioIndexToConstraintIndex(s, v)]);
}

Eigen::Vector3d SMPCC::getScenarioLocation(const int &k, const Scenario &scenario)
{

  return Eigen::Vector3d((*scenarios_)[k][scenario.obstacle_idx_][0](scenario.idx_), (*scenarios_)[k][scenario.obstacle_idx_][1](scenario.idx_), 0.2);
}

void SMPCC::publishVisuals()
{
  // Draws all the markers added to ros_markers_ and resets
  ros_markers_->publish();
}