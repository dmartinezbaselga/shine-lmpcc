#include "modules_objectives/guidance_objective.h"

#include <lmpcc/lmpcc_configuration.h>
#include <lmpcc/PredictiveControllerConfig.h>
#include <lmpcc_solver/SolverInclude.h>

#include <algorithm>

GuidanceObjective::LocalPlanner::LocalPlanner(int _id, ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle, bool _is_original_planner)
    : id(_id), is_original_planner(_is_original_planner)
{

  reference_velocities_.resize(config->N);
  solver.reset(new SolverInterface(_id));
}

GuidanceObjective::GuidanceObjective(ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle)
    : ReferencePath(nh, config, vehicle)
{
  time_sub_ = nh.subscribe("/time", 1, &GuidanceObjective::TimeCallback, this);
  LMPCC_WARN_ALWAYS("Initializing Guidance Objective Module");
  type_ = ModuleType::OBJECTIVE; // Depends on what we are using the guidance for

  config_->N_pedestrians_ = GuidancePlanner::Config::N; // Increase the horizon expected for pedestrians to the PRM horizon

  plan_markers_.reset(new RosTools::ROSMarkerPublisher(nh, "guidance_planner/optimized_plans", config_->target_frame_, 450));

  // Initialize the constraint modules
  int n_solvers = global_guidance_.GetConfig()->n_paths_; // + 1 for the main lmpcc solver?
  for (int i = 0; i < n_solvers; i++)
  {
    planners_.emplace_back(i, nh, config_, vehicle);
    signal_publishers_.emplace_back(nh, "gmpcc_" + std::to_string(i));
  }

  if (config_->guidance_add_original_problem_) // Add a planner that does not use the guidance
  {
    LMPCC_WARN_ALWAYS("GMPCC: Adding the non-guided planner in parallel (see guidance/constraints/add_original_problem_)");
    planners_.emplace_back(n_solvers, nh, config_, vehicle, true);
    signal_publishers_.emplace_back(nh, "lmpcc");
  }

  global_guidance_.SetTrackOnlyTheSelectedHomology(); // We only care about the selected trajectory
  global_guidance_.SetReferenceVelocity(config_->reference_velocity_);

  LMPCC_SUCCESS_ALWAYS("Guidance Objective Initialized")
}

void GuidanceObjective::TimeCallback(const std_msgs::Time::ConstPtr &msg){
  this->last_time_received_ = *msg;
  this->receiving_times_ = true;
}

void GuidanceObjective::GetMethodName(std::string &name)
{
  if (!config_->guidance_add_original_problem_)
    name = "Guidance-MPCCNO";
  else
    name = "Guidance-MPCC";
}

void GuidanceObjective::Update(SolverInterface *solver_interface, RealTimeData &data)
{
  PROFILE_AND_LOG(config_->debug_output_, "Guidance Objective: Update()");
  parameter_start_index_ = -1;
  data_ptr_ = &data;

  // First run the base class update to update our progress along the path
  ReferencePath::Update(solver_interface, data);

  // If the guidance should run
  if (!(config_->guidance_add_original_problem_ && global_guidance_.GetConfig()->n_paths_ == 0))
  {
    // Set the goals of the guidance planner
    if (!config_->legacy_icra_mode_) // Goal grid
    {
      if (config_->use_goal_grid_){
        global_guidance_.SetGoalGrid(goal_, 2.0, 2.0);
      }
      else{
        double road_width_left = config_->two_way_road_ ? config_->road_width_left_ * 3. : config_->road_width_left_;
        global_guidance_.LoadReferencePath(std::max(0., minimal_s_), reference_path_,
                                          road_width_left - vehicle_->DiscRadius() - 0.1,
                                          config_->road_width_right_ - vehicle_->DiscRadius() - 0.1);
      }
    }
    else // Single goal
    {
      double s_goal = std::max(0., minimal_s_) +
                      global_guidance_.GetConfig()->DT * (double)global_guidance_.GetConfig()->N * global_guidance_.GetConfig()->reference_velocity_;
      global_guidance_.SetGoals({GuidancePlanner::Goal(Eigen::Vector2d(reference_path_->GetPoint(s_goal)), 0.)});
    }

    LMPCC_INFO("Running Guidance Search");
    global_guidance_.Update(); /** @note The main update */
    LMPCC_INFO_STREAM("Guidance Search found " << global_guidance_.NumberOfGuidanceTrajectories() << " guidance trajectories.");
  }
}

void GuidanceObjective::InitializeSolverWithGuidance(LocalPlanner &planner)
{
  // Initialize the solver with the guidance trajectory
  RosTools::CubicSpline2D<tk::spline> &trajectory_spline = global_guidance_.GetGuidanceTrajectory(planner.id).spline.GetTrajectory();

  // Initialize the solver in the selected local optimum
  // I.e., set for each k, x(k), y(k) ...
  for (size_t k = 0; k < planner.solver->FORCES_N; k++) // note that the 0th velocity is the current velocity
  {
    int index = k + 1;
    Eigen::Vector2d cur_position = trajectory_spline.GetPoint((double)(index)*planner.solver->DT);    // The plan is one ahead
    Eigen::Vector2d cur_velocity = trajectory_spline.GetVelocity((double)(index)*planner.solver->DT); // The plan is one ahead

    planner.solver->InitialPlan(k).set_x(cur_position(0));
    planner.solver->InitialPlan(k).set_y(cur_position(1));
    planner.solver->InitialPlan(k).set_psi(std::atan2(cur_velocity(1), cur_velocity(0)));
    planner.solver->InitialPlan(k).set_v(cur_velocity.norm());
  }
}

void GuidanceObjective::LoadGuidanceTrajectory(LocalPlanner &planner)
{
  LMPCC_INFO_STREAM("Guidance Objective::LoadGuidanceTrajectory (planner ID: " << planner.id << ")");

  if (config_->use_guidance_as_reference_ && !planner.is_original_planner && global_guidance_.Succeeded())
  {

    auto &guidance_trajectory = global_guidance_.GetGuidanceTrajectory(planner.id).spline;

    // We start at the start of the path
    planner.solver->setInitialSpline(0.);
    planner.solver->spline(0) = 0.;

    // Retrieve the path of the guidance trajectory
    planner.guidance_path_.reset(new RosTools::CubicSpline2D<tk::spline>(guidance_trajectory.GetPath()));

    // Retrieve the velocities along the guidance trajectory
    for (size_t k = 0; k < planner.solver->FORCES_N; k++)
    {
      double t = ((double)(k)) * global_guidance_.GetConfig()->DT;
      planner.reference_velocities_[k] = guidance_trajectory.GetTrajectory().GetVelocity(t).norm();
    }
  }
  else // Insert the regular path and velocities
  {
    // We start at where we are on the reference path
    planner.solver->setInitialSpline(minimal_s_);
    planner.solver->spline(0) = minimal_s_;

    planner.guidance_path_.reset(new RosTools::CubicSpline2D<tk::spline>(*reference_path_));

    for (size_t k = 0; k < planner.solver->FORCES_N; k++)
      planner.reference_velocities_[k] = global_guidance_.GetConfig()->reference_velocity_;
  }
}

void GuidanceObjective::SetParameters(SolverInterface *solver, const RealTimeData &data, int N_iter, int &param_idx)
{
  if (N_iter == 0)
    LMPCC_INFO("Guidance Objective::Set Parameters (GLOBAL)");

  // Save where to insert the data, so we can repeat it for the other solvers
  if (parameter_start_index_ == -1)
    parameter_start_index_ = param_idx;

  solver->setTimeout(config_->solver_timeout_ / 1000.); // Limit the solver times
  SetParameters(planners_[0], data, N_iter, param_idx); // To fill the right number of parameters
}

void GuidanceObjective::SetParameters(LocalPlanner &planner, const RealTimeData &data, int N_iter, int &param_idx)
{
  if (N_iter == 0)
  {
    LMPCC_INFO("Guidance Objective::SetParameters (planner ID: " << planner.id << ")");
    LoadGuidanceTrajectory(planner);
    planner.solver->setTimeout(1. / (config_->clock_frequency_) - (ros::Time::now() - data.control_loop_time_).toSec() - 0.005);
  }

  std::unique_ptr<RosTools::Spline2D> &guidance_path = planner.guidance_path_;

  //  For tracking the guidance path: we always start at index = 0, because we computed it from the robot, otherwise use the global progress
  int offset = (config_->use_guidance_as_reference_ && !planner.is_original_planner) ? 0 : spline_index_;
  for (int segment_id = 0; segment_id < planner.solver->n_segments_; segment_id++)
  {
    double ax, ay, bx, by, cx, cy, dx, dy;

    if (config_->use_guidance_as_reference_) // Get parameters in this segment from the guidance path
      guidance_path->GetParameters(std::min(offset + segment_id, guidance_path->NumberOfSegments() - 1), ax, bx, cx, dx, ay, by, cy, dy);
    else // Get parameters from the regular reference path
      reference_path_->GetParameters(std::min(offset + segment_id, planner.solver->n_segments_), ax, bx, cx, dx, ay, by, cy, dy);

    planner.solver->setParameter(N_iter, param_idx, ax); // spline coefficients
    planner.solver->setParameter(N_iter, param_idx, bx); // spline coefficients
    planner.solver->setParameter(N_iter, param_idx, cx); // spline coefficients
    planner.solver->setParameter(N_iter, param_idx, dx); // spline coefficients

    planner.solver->setParameter(N_iter, param_idx, ay); // spline coefficients
    planner.solver->setParameter(N_iter, param_idx, by); // spline coefficients
    planner.solver->setParameter(N_iter, param_idx, cy); // spline coefficients
    planner.solver->setParameter(N_iter, param_idx, dy); // spline coefficients

    if (config_->use_guidance_as_reference_) // Get the start of this segment
      planner.solver->setParameter(N_iter, param_idx, guidance_path->GetSplineStart(std::min(offset + segment_id, guidance_path->NumberOfSegments() - 1)));
    else // Get the start of the global path segment
      planner.solver->setParameter(N_iter, param_idx, reference_path_->GetSplineStart(std::min(planner.solver->n_segments_, offset + segment_id)));
  }

  /* Insert the velocity reference */
  auto &velocities = planner.reference_velocities_;
  int velocity_index = std::max(0, std::min(N_iter - 1, (int)velocities.size() - 1));

  planner.solver->setParameter(N_iter, param_idx, velocities[velocity_index]);
}

int GuidanceObjective::Optimize(SolverInterface *solver_interface)
{
  // Required for parallel call to the solvers when using Forces
  // omp_set_nested(1);
  // omp_set_max_active_levels(2);
  // omp_set_dynamic(0);

  if (!config_->guidance_add_original_problem_ && !global_guidance_.Succeeded())
  {
    std::cout << "global guidance did not succeed, returning infeasible" << std::endl;
    return -1;
  }
  // #pragma omp parallel for num_threads(8)
  // for (auto &planner : planners_)
  // {
  auto &planner = planners_[0];
  planner.result.Reset();

  planner.disabled = false;
  // if (planner.id >= global_guidance_.NumberOfGuidanceTrajectories())
  // {
  //   if (!planner.is_original_planner) // We still want to add the original planner!
  //   {
  //     planner.disabled = true;
  //     continue;
  //   }
  // }
  auto &solver = planner.solver;
  PROFILE_SCOPE("Parallel Guidance Optimization");
  LMPCC_INFO("Planner [" << planner.id << "]: Copying data from main solver");
  solver->CopySolverParameters(solver_interface); // This makes the current solver equal to the main solver

  if (!planner.is_original_planner) // If there is a guidance trajectory to initialize from
  {
    LMPCC_INFO("Planner [" << planner.id << "]: Loading guidance path and velocities into the solver");
    InitializeSolverWithGuidance(planner);
    solver->LoadInitialVehiclePredictions(); // Load vehicle predictions for constraint construction

    if (config_->enable_guidance_warmstart_)
      solver->loadInitialPlanAsWarmStart(); // Warmstart if configured
  }

  // LOAD PARAMETERS
  LMPCC_INFO("Planner [" << planner.id << "]: Loading updated parameters into the solver");
  for (size_t N_iter = 0; N_iter < solver->FORCES_NBAR; N_iter++)
  {
    int param_idx = parameter_start_index_;                // Copy, so that it is not increased
    SetParameters(planner, *data_ptr_, N_iter, param_idx); // Set this solver's parameters
  }

  // SOLVE OPTIMIZATION
  LMPCC_INFO("Planner [" << planner.id << "]: Solving ...");
  planner.result.exit_code = solver->solve();
  LMPCC_INFO("Planner [" << planner.id << "]: Done! (exitcode = " << planner.result.exit_code << ")");

  // ANALYSIS AND PROCESSING
  planner.result.success = planner.result.exit_code == 1;
  planner.result.objective = solver->forces_info_.pobj; // How good is the solution?

  if (planner.is_original_planner) // We did not use any guidance!
  {
    planner.result.guidance_ID = global_guidance_.GetConfig()->n_paths_ * 2; // one higher than the number of topology IDs
  }
  else
  {
    planner.result.guidance_ID = global_guidance_.GetGuidanceTrajectory(planner.id).topology_class; // We were using this guidance

    // if (planner.result.guidance_ID == global_guidance_.GetUsedTrajectory())                           // Prefer the selected trajectory
    // planner.result.objective *= (1. - global_guidance_.GetConfig()->selection_weight_consistency_); // Discount percentage wise
  }
  // }

  // omp_set_dynamic(1);

  // DECISION MAKING
  // Can still be improved, here we use the ICRA version: Pick the best guidance trajectory
  // if (global_guidance_.Succeeded())
  //   std::cout << "succes\n";
  // else
  //   std::cout << "failure\n";
  if (global_guidance_.Succeeded() || !config_->guidance_add_original_problem_)
    best_planner_index_ = 0; // FindBestPlanner();
  else if (config_->guidance_add_original_problem_)
    best_planner_index_ = global_guidance_.GetConfig()->n_paths_; // If there was no guidance, use the unguided version instead

  if (best_planner_index_ == -1)
  {
    LMPCC_WARN("Failed to find a feasible trajectory in any of the " << planners_.size() << " optimizations.");
    return planners_[0].result.exit_code;
  }

  auto &best_planner = planners_[best_planner_index_];
  auto &best_solver = best_planner.solver;
  bool original_plan_is_best = best_planner.is_original_planner;
  LMPCC_INFO("Best Planner ID: " << best_planner.id);

  // VISUALIZATION
  // if (!original_plan_is_best)
  // global_guidance_.SetUsedTrajectory(best_planner.result.guidance_ID); // Communicate to the guidance which plan was best

  solver_interface->CopySolverOutput(best_solver.get()); // Load the solution into the main lmpcc solver

  // Now that we have copied the best solver data (which will get propagated), propagate the solvers here
  for (auto &planner : planners_)
  {
    if (planner.result.success)
      planner.solver->LoadSolution(config_->shift_plan_forward_);
  }

  return best_planner.result.exit_code; // Return its exit code*/
}

/** @brief Load obstacles into the Homotopy module */
void GuidanceObjective::OnDataReceived(SolverInterface *solver_interface, RealTimeData &data, std::string &&data_name)
{

  // See if there is any data for the reference path
  ReferencePath::OnDataReceived(solver_interface, data, std::forward<std::string>(data_name));

  // We wait for both the obstacles and state to arrive before we compute here
  if (data_name == "Dynamic Obstacles")
  {
    LMPCC_INFO("Guidance Constraints: Received dynamic obstacles")
    std::vector<GuidancePlanner::Obstacle> obstacles;
    for (auto &obstacle : data.dynamic_obstacles_)
    {
      std::vector<Eigen::Vector2d> positions;
      size_t k;
      positions.push_back(Eigen::Vector2d(obstacle.pose_.position.x, obstacle.pose_.position.y));

      for (k = 0; k < std::max(obstacle.prediction_.gaussians[0].mean.poses.size(), (size_t)GuidancePlanner::Config::N); k++)
      {
        // std::cout << "[" << k << "]: (" << obstacle.prediction_.gaussians[0].mean.poses[k].pose.position.x << ", " << obstacle.prediction_.gaussians[0].mean.poses[k].pose.position.y << std::endl;
        positions.push_back(Eigen::Vector2d(obstacle.prediction_.gaussians[0].mean.poses[k].pose.position.x,
                                            obstacle.prediction_.gaussians[0].mean.poses[k].pose.position.y));
      }

      obstacles.emplace_back(obstacle.id_, positions, obstacle.discs_[0].radius + vehicle_->discs_[0].radius);
    }
    if (receiving_times_){
      global_guidance_.LoadObstacles(obstacles, data.halfspaces_[0], &last_time_received_.data);
    }
    else{
      global_guidance_.LoadObstacles(obstacles, data.halfspaces_[0]);
    }
  }
  else if (data_name == "State")
  {
    if (receiving_times_){
      global_guidance_.SetStart(Eigen::Vector2d(solver_interface->State().x(), solver_interface->State().y()), solver_interface->State().psi(),
                              solver_interface->State().v(), &last_time_received_.data);
    }
    else{
        global_guidance_.SetStart(Eigen::Vector2d(solver_interface->State().x(), solver_interface->State().y()), solver_interface->State().psi(),
                              solver_interface->State().v());
    }
  }
  else if (data_name == "Goal")
  {
    LMPCC_WARN("Guidance: Received a new goal");

    goal_ = data.goal_;
    goal_received_ = true;
  }
}

void GuidanceObjective::OnReset(SolverInterface *solver_interface)
{
  ReferencePath::OnReset(solver_interface);

  global_guidance_.Reset();
}

void GuidanceObjective::ReconfigureCallback(SolverInterface *solver_interface, lmpcc::PredictiveControllerConfig &config,
                                            uint32_t level, bool first_callback)
{
  if (first_callback)
  {
    config.spline_consistency = global_guidance_.GetConfig()->selection_weight_consistency_;
    config.highlight_selected = config_->highlight_selected_guidance_;
  }
  else
  {
    global_guidance_.GetConfig()->selection_weight_consistency_ = config.spline_consistency;
    config_->highlight_selected_guidance_ = config.highlight_selected;
  }

  global_guidance_.SetReferenceVelocity(config.velocity_reference);
  config_->visualized_guidance_trajectory_nr_ = config.visualize_trajectory_nr;
}

void GuidanceObjective::ExportData(RosTools::DataSaver &data_saver)
{
  LMPCC_INFO("Guidance Objective: ExportData()");

  data_saver.AddData("runtime_guidance", global_guidance_.GetLastRuntime());
  global_guidance_.ExportData(data_saver);
}

/** @brief Visualize the computations in this module  */
void GuidanceObjective::Visualize()
{
  PROFILE_AND_LOG(config_->debug_output_, "Guidance Objective: Visualize()");

  if (!global_guidance_.Succeeded())
    return;
  ReferencePath::Visualize();

  global_guidance_.Visualize(config_->highlight_selected_guidance_, config_->visualized_guidance_trajectory_nr_);

  for (auto &planner : planners_)
  {
    if (planner.disabled)
      continue;

    if (planner.result.success)
      VisualizeOptimizedPlan(planner); // Visualize the plan
  }

  plan_markers_->publish();

  // Visualize the objectives
  for (size_t i = 0; i < signal_publishers_.size(); i++)
  {
    if (planners_[i].disabled || (!planners_[i].result.success))
      signal_publishers_[i].Publish(std::numeric_limits<double>::quiet_NaN());
    else
      signal_publishers_[i].Publish(planners_[i].result.objective);
  }
}

void GuidanceObjective::VisualizeOptimizedPlan(LocalPlanner &planner)
{
  RosTools::ROSPointMarker &plan_points = plan_markers_->getNewPointMarker("CYLINDER");
  RosTools::ROSPointMarker &ellipse = plan_markers_->getNewPointMarker("CYLINDER");
  RosTools::ROSLine &line = plan_markers_->getNewLine();

  bool is_selected_solver = planner.id == best_planner_index_;

  plan_points.setScale(0.2 * config_->scenario_visual_scale_, 0.2 * config_->scenario_visual_scale_, 0.1e-3);
  ellipse.setScale(2 * vehicle_->discs_[0].radius, 2 * vehicle_->discs_[0].radius);
  line.setScale(0.15 * config_->scenario_visual_scale_, 0.15 * config_->scenario_visual_scale_);

  if (is_selected_solver && config_->highlight_selected_guidance_)
  {
    VisualizeGMPCCPlan(planner, plan_points, ellipse, line);
  }
  else
  {
    if (planner.is_original_planner)
      VisualizeLMPCCPlan(planner, plan_points, ellipse, line);
    else
      VisualizeGuidedPlan(planner, plan_points, ellipse, line);
  }

  if (config_->visualize_warmstart_)
    VisualizeWarmstartPlan(planner, plan_points, ellipse, line);
}

void GuidanceObjective::VisualizeWarmstartPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line)
{
  if (config_->visualized_guidance_trajectory_nr_ != -1 && config_->visualized_guidance_trajectory_nr_ != planner.id)
    return;

  plan_points.setColor(0., 1., 0.);
  ellipse.setColor(0., 1., 0.);
  line.setColor(0., 1., 0.);

  auto &vehicle_regions = planner.solver->InitialVehiclePrediction(); // Get the ego-vehicle predictions
  Eigen::Vector2d pose, previous_pose;

  double z = 0.6e-1;

  for (size_t k = 0; k < vehicle_regions.size(); k++)
  {
    for (auto &disc : vehicle_regions[k].discs_)
    {
      plan_points.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z));
      ellipse.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z - 2e-3));

      if (k >= 1)
        line.addLine(Eigen::Vector3d(previous_pose(0), previous_pose(1), z - 1e-3), Eigen::Vector3d(disc.x, disc.y, z - 1e-3));

      previous_pose = disc.AsVector2d();
    }
  }
}

void GuidanceObjective::VisualizeGuidedPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line)
{
  if (config_->visualized_guidance_trajectory_nr_ != -1 && config_->visualized_guidance_trajectory_nr_ != planner.id)
    return;

  int color_idx = planner.result.guidance_ID;
  double alpha = 0.15;

  plan_points.setColorInt(color_idx, global_guidance_.NumberOfGuidanceTrajectories());
  ellipse.setColorInt(color_idx, global_guidance_.NumberOfGuidanceTrajectories(), alpha);
  line.setColorInt(color_idx, global_guidance_.NumberOfGuidanceTrajectories());

  auto &vehicle_regions = planner.solver->OptimizedVehiclePredictions(); // Get the ego-vehicle predictions
  Eigen::Vector2d pose, previous_pose;

  double z = 0.6e-1;

  for (size_t k = 0; k < vehicle_regions.size(); k++)
  {
    for (auto &disc : vehicle_regions[k].discs_)
    {
      plan_points.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z));
      ellipse.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z - 2e-3));

      if (k >= 1)
        line.addLine(Eigen::Vector3d(previous_pose(0), previous_pose(1), z - 1e-3), Eigen::Vector3d(disc.x, disc.y, z - 1e-3));

      previous_pose = disc.AsVector2d();
    }
  }
}

void GuidanceObjective::VisualizeGMPCCPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line)
{
  if (config_->visualized_guidance_trajectory_nr_ != -1 && config_->visualized_guidance_trajectory_nr_ != planner.id)
    return;

  plan_points.setColorInt(2, 0.8, RosTools::Colormap::BRUNO);
  ellipse.setColorInt(2, 0.8, RosTools::Colormap::BRUNO);
  line.setColorInt(2, 0.8, RosTools::Colormap::BRUNO);

  auto &vehicle_regions = planner.solver->OptimizedVehiclePredictions(); // Get the ego-vehicle predictions
  Eigen::Vector2d pose, previous_pose;

  double z = 0.2;

  for (size_t k = 0; k < vehicle_regions.size(); k++)
  {
    for (auto &disc : vehicle_regions[k].discs_)
    {
      plan_points.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z));
      ellipse.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z - 2e-3));

      if (k >= 1)
        line.addLine(Eigen::Vector3d(previous_pose(0), previous_pose(1), z - 1e-3), Eigen::Vector3d(disc.x, disc.y, z - 1e-3));

      previous_pose = disc.AsVector2d();
    }
  }
}

void GuidanceObjective::VisualizeLMPCCPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line)
{

  double alpha = 0.6;

  plan_points.setColorInt(3, alpha, RosTools::Colormap::BRUNO);
  ellipse.setColorInt(3, alpha, RosTools::Colormap::BRUNO);
  line.setColorInt(3, alpha, RosTools::Colormap::BRUNO);

  auto &vehicle_regions = planner.solver->OptimizedVehiclePredictions(); // Get the ego-vehicle predictions
  Eigen::Vector2d pose, previous_pose;

  double z = 0.5;

  for (size_t k = 0; k < vehicle_regions.size(); k++)
  {
    for (auto &disc : vehicle_regions[k].discs_)
    {
      plan_points.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z));
      // ellipse.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z - 2e-3));

      if (k >= 1)
        line.addLine(Eigen::Vector3d(previous_pose(0), previous_pose(1), z - 1e-3), Eigen::Vector3d(disc.x, disc.y, z - 1e-3));

      previous_pose = disc.AsVector2d();
    }
  }
}