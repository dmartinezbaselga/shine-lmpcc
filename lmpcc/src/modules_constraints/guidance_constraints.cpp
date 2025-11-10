#include "modules_constraints/guidance_constraints.h"

#include <lmpcc/lmpcc_configuration.h>
#include <lmpcc/PredictiveControllerConfig.h>
#include <lmpcc_solver/SolverInclude.h>

GuidanceConstraints::LocalPlanner::LocalPlanner(int _id, ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle, bool _is_original_planner)
    : id(_id), is_original_planner(_is_original_planner)
{
  guidance_constraints.reset(new LinearizedConstraints(nh, config, vehicle));
  safety_constraints.reset(new GUIDANCE_CONSTRAINTS_TYPE(nh, config, vehicle));
  solver.reset(new SolverInterface(_id));
}

GuidanceConstraints::GuidanceConstraints(ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle)
    : ReferencePath(nh, config, vehicle)
{
  LMPCC_WARN_ALWAYS("Initializing Guidance Constraints Module");
  type_ = ModuleType::CONSTRAINT; // Depends on what we are using the guidance for

  config_->N_pedestrians_ = GuidancePlanner::Config::N; // Increase the horizon expected for pedestrians to the PRM horizon

  plan_markers_.reset(new RosTools::ROSMarkerPublisher(nh, "guidance_planner/optimized_plans", config_->target_frame_, 450));

  // Initialize the constraint modules
  int n_solvers = global_guidance_.GetConfig()->n_paths_; // + 1 for the main lmpcc solver?

  for (int i = 0; i < n_solvers; i++)
  {
    planners_.emplace_back(i, nh, config_, vehicle);
    signal_publishers_.emplace_back(nh, "gmpcc_" + std::to_string(i));
  }

  if (config_->guidance_add_original_problem_) // ADD IT AS FIRST PLAN
  {
    LMPCC_WARN_ALWAYS("GMPCC: Adding the non-guided planner in parallel (see guidance/constraints/add_original_problem_)");
    planners_.emplace_back(n_solvers, nh, config_, vehicle, true);

    signal_publishers_.emplace_back(nh, "lmpcc");
  }

  LMPCC_SUCCESS_ALWAYS("Guidance Constraints Initialized")
}

void GuidanceConstraints::GetMethodName(std::string &name)
{
  if (config_->guidance_add_original_problem_ && global_guidance_.GetConfig()->n_paths_ == 0)
  {
    name = "LMPCC";
  }
  else
  {
    if (!config_->guidance_add_original_problem_)
      name = "GMPCCNO";
    else
      name = "GMPCC";
  }
}

void GuidanceConstraints::Update(SolverInterface *solver_interface, RealTimeData &data)
{
  PROFILE_AND_LOG(config_->debug_output_, "Guidance Constraints: Update()");
  parameter_start_index_ = -1;
  data_ptr_ = &data;

  // First run the base class update to update our progress along the path
  ReferencePath::Update(solver_interface, data);

  // global_guidance_.LoadHalfspaces(); // Load static obstacles represented by halfspaces

  // Is this necessary?
  planners_[0].guidance_constraints->Update(solver_interface, data);
  planners_[0].safety_constraints->Update(solver_interface, data);

  if (!(config_->guidance_add_original_problem_ && global_guidance_.GetConfig()->n_paths_ == 0)) // In this case we are not using guidance
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

    LMPCC_INFO("Running Guidance Search");
    global_guidance_.Update(); // data); /** @note The main update */
    LMPCC_INFO_STREAM("Guidance Search found " << global_guidance_.NumberOfGuidanceTrajectories() << " guidance trajectories.");
  }

  // Initialize the solver with the best (= 0) guidance trajectory, if configured
  // if (config_->enable_guidance_)
  // InitializeSolverWithGuidance(solver_interface);
}

void GuidanceConstraints::InitializeSolverWithGuidance(SolverInterface *solver)
{
  // Initialize the solver with the guidance trajectory
  // int trajectory_index = solver->solver_id_ == 0 ? 0 : solver->solver_id_ - 1;
  RosTools::CubicSpline2D<tk::spline> &trajectory_spline = global_guidance_.GetGuidanceTrajectory(solver->solver_id_).spline.GetTrajectory();

  // Initialize the solver in the selected local optimum
  // I.e., set for each k, x(k), y(k) ...
  for (size_t k = 0; k < solver->FORCES_N; k++) // note that the 0th velocity is the current velocity
  {
    int index = k + 1;
    // if (config_->debug_boolean1_)
    //   index = k;
    Eigen::Vector2d cur_position = trajectory_spline.GetPoint((double)(index)*solver->DT); // The plan is one ahead
    // global_guidance_.ProjectToFreeSpace(cur_position, k + 1);

    Eigen::Vector2d cur_velocity = trajectory_spline.GetVelocity((double)(index)*solver->DT); // The plan is one ahead
    solver->InitialPlan(k).set_x(cur_position(0));
    solver->InitialPlan(k).set_y(cur_position(1));
    solver->InitialPlan(k).set_psi(std::atan2(cur_velocity(1), cur_velocity(0)));
    solver->InitialPlan(k).set_v(cur_velocity.norm());
  }
}

void GuidanceConstraints::SetParameters(LocalPlanner &planner, const RealTimeData &data, int N_iter, int &param_idx)
{
  // Tell each solver how much time it has

  planner.solver->setTimeout(1. / (config_->clock_frequency_) - (ros::Time::now() - data.control_loop_time_).toSec() - 0.005);
  // config_->solver_timeout_ / 1000.); // Limit the solver to 40 ms

  // Load parameters

  planner.guidance_constraints->SetParameters(planner.solver.get(), data, N_iter, param_idx);
  planner.safety_constraints->SetParameters(planner.solver.get(), data, N_iter, param_idx);
}

void GuidanceConstraints::SetParameters(SolverInterface *solver, const RealTimeData &data, int N_iter, int &param_idx)
{
  if (N_iter == 1)
    LMPCC_INFO("Guidance Constraints::SetParameters");

  // Save where to insert the data, so we can repeat it for the other solvers
  if (parameter_start_index_ == -1)
  {
    parameter_start_index_ = param_idx;
  }

  solver->setTimeout(config_->solver_timeout_ / 1000.); // Limit the solver to 40 ms

  SetParameters(planners_[0], data, N_iter, param_idx); // For the first planner!
}

int GuidanceConstraints::Optimize(SolverInterface *solver_interface)
{
  // Required for parallel call to the solvers when using Forces
  omp_set_nested(1);
  omp_set_max_active_levels(2);
  omp_set_dynamic(0);

  if (!config_->guidance_add_original_problem_ && !global_guidance_.Succeeded())
    return 0;

#pragma omp parallel for num_threads(8)
  for (auto &planner : planners_)
  {
    planner.result.Reset();

    planner.disabled = false;
    if (planner.id >= global_guidance_.NumberOfGuidanceTrajectories())
    {
      if (!planner.is_original_planner) // We still want to add the original planner!
      {
        planner.disabled = true;
        continue;
      }
    }
    auto &solver = planner.solver;
    PROFILE_SCOPE("Parallel Guidance Optimization");
    LMPCC_INFO("Planner [" << planner.id << "]: Copying data from main solver");
    solver->CopySolverParameters(solver_interface); // This makes the current solver equal to the main solver

    // CONSTRUCT CONSTRAINTS
    if (planner.is_original_planner || (!config_->enable_guidance_constraints_))
    {
      // For the original problem we construct dummy constraints but with the static constraints (no guidance!)
      LMPCC_INFO("Planner [" << planner.id << "]: Constructing dummy constraints");
      empty_data_.halfspaces_ = data_ptr_->halfspaces_; // Copy in the halfspace data

      // solver->LoadInitialVehiclePredictions(); Note: we do not care about the initial position
      // solver->LoadVehiclePredictionsToInitialPlan
      planner.guidance_constraints->Update(solver.get(), empty_data_);
      planner.safety_constraints->Update(solver.get(), *data_ptr_); // Updates collision avoidance constraints
    }
    else
    {
      LMPCC_INFO("Planner [" << planner.id << "]: Loading guidance into the solver and constructing constraints");
      InitializeSolverWithGuidance(solver.get());
      solver->LoadInitialVehiclePredictions(); // Load vehicle predictions for constraint construction

      planner.guidance_constraints->Update(solver.get(), *data_ptr_); // Updates linearization of constraints
      planner.safety_constraints->Update(solver.get(), *data_ptr_);   // Updates collision avoidance constraints
    }

    // LOAD PARAMETERS
    if (config_->enable_guidance_warmstart_)
      solver->loadInitialPlanAsWarmStart(); // Warmstart if configured

    LMPCC_INFO("Planner [" << planner.id << "]: Loading updated parameters into the solver");
    for (size_t N_iter = 1; N_iter < solver->FORCES_NBAR - 1; N_iter++)
    {
      int param_idx = parameter_start_index_;                // Copy, so that it is not increased
      SetParameters(planner, *data_ptr_, N_iter, param_idx); // Set this solver's parameters
    }

    // SOLVE OPTIMIZATION
    LMPCC_INFO("Planner [" << planner.id << "]: Solving ...");
    planner.result.exit_code = solver->solve();
    // solver_results_[i].exit_code =ellipsoidal_constraints_[solver->solver_id_].Optimize(solver.get()); // IF THIS OPTIMIZATION EXISTS!
    LMPCC_INFO("Planner [" << planner.id << "]: Done! (exitcode = " << planner.result.exit_code << ")");

    // ANALYSIS AND PROCESSING
    planner.result.success = planner.result.exit_code == 1;
    planner.result.objective = solver->forces_info_.pobj; // How good is the solution?

    if (planner.is_original_planner) // We did not use any guidance!
    {
      planner.result.guidance_ID = 2 * global_guidance_.GetConfig()->n_paths_; // one higher than the maximum number of topology classes
    }
    else
    {
      auto &guidance_trajectory = global_guidance_.GetGuidanceTrajectory(planner.solver->solver_id_);
      planner.result.guidance_ID = guidance_trajectory.topology_class; // We were using this guidance

      if (guidance_trajectory.previously_selected_) // Prefer the selected trajectory
        planner.result.objective *= 1. / global_guidance_.GetConfig()->selection_weight_consistency_;
      // planner.result.objective *= (1. - global_guidance_.GetConfig()->selection_weight_consistency_); // Discount percentage wise
    }
  }

  omp_set_dynamic(1);

  // DECISION MAKING
  best_planner_index_ = FindBestPlanner();
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
  best_planner.guidance_constraints->Visualize();
  best_planner.safety_constraints->Visualize();
  // Communicate to the guidance which topology class we follow (none if it was the original planner)
  global_guidance_.OverrideSelectedTrajectory(best_planner.result.guidance_ID, best_planner.is_original_planner);

  solver_interface->CopySolverOutput(best_solver.get()); // Load the solution into the main lmpcc solver

  // Now that we have copied the best solver data (which will get propagated), propagate the solvers here
  for (auto &planner : planners_)
  {
    if (planner.result.success)
      planner.solver->LoadSolution(config_->shift_plan_forward_);
  }

  return best_planner.result.exit_code; // Return its exit code
}

int GuidanceConstraints::FindBestPlanner()
{
  // Find the best feasible solution
  double best_solution = 1e10;
  int best_index = -1;
  for (size_t i = 0; i < planners_.size(); i++)
  {
    auto &planner = planners_[i];
    if (planner.disabled) // Do not consider disabled planners
      continue;

    if (planner.result.success && planner.result.objective < best_solution)
    {
      best_solution = planner.result.objective;
      best_index = i;
    }
  }
  return best_index;
};

/** @brief Visualize the computations in this module  */
void GuidanceConstraints::Visualize()
{
  PROFILE_AND_LOG(config_->debug_output_, "Guidance Constraints: Visualize()");

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

void GuidanceConstraints::VisualizeOptimizedPlan(LocalPlanner &planner)
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

void GuidanceConstraints::VisualizeWarmstartPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line)
{
  if (config_->visualized_guidance_trajectory_nr_ != -1 && config_->visualized_guidance_trajectory_nr_ != planner.id)
    return;

  int color_idx = planner.result.guidance_ID;
  double alpha = 1.0;

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

void GuidanceConstraints::VisualizeGuidedPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line)
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

void GuidanceConstraints::VisualizeGMPCCPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line)
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

void GuidanceConstraints::VisualizeLMPCCPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line)
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

/** @brief Load obstacles into the Homotopy module */
void GuidanceConstraints::OnDataReceived(SolverInterface *solver_interface, RealTimeData &data, std::string &&data_name)
{
  // See if there is any data for the reference path
  ReferencePath::OnDataReceived(solver_interface, data, std::forward<std::string>(data_name));

  // We wait for both the obstacles and state to arrive before we compute here
  if (data_name == "Dynamic Obstacles")
  {
    LMPCC_INFO("Guidance Constraints: Received dynamic obstacles")
#pragma omp parallel for num_threads(8)
    for (auto &planner : planners_)
    {
      planner.safety_constraints->OnDataReceived(planner.solver.get(), data, std::forward<std::string>(data_name));
    }

    std::vector<GuidancePlanner::Obstacle> obstacles;
    for (auto &obstacle : data.dynamic_obstacles_)
    {
      std::vector<Eigen::Vector2d> positions;
      size_t k;
      positions.push_back(Eigen::Vector2d(obstacle.pose_.position.x, obstacle.pose_.position.y)); // Insert this at k = 0, so that
                                                                                                  // we can plan from k=1 onwards

      // If we are computing for the next step, then ignore the first prediction?

      for (k = 0; k < std::max(obstacle.prediction_.gaussians[0].mean.poses.size(), (size_t)GuidancePlanner::Config::N); k++)
      {
        // std::cout << "[" << k << "]: (" << obstacle.prediction_.gaussians[0].mean.poses[k].pose.position.x << ", " << obstacle.prediction_.gaussians[0].mean.poses[k].pose.position.y << std::endl;
        positions.push_back(Eigen::Vector2d(obstacle.prediction_.gaussians[0].mean.poses[k].pose.position.x,
                                            obstacle.prediction_.gaussians[0].mean.poses[k].pose.position.y));
      }

      obstacles.emplace_back(obstacle.id_, positions, obstacle.discs_[0].radius + vehicle_->discs_[0].radius); /** @todo Include the robot
                                                                                                                    position */
    }

    global_guidance_.LoadObstacles(obstacles, data.halfspaces_[0]);
  }
  else if (data_name == "State")
  {
    global_guidance_.SetStart(Eigen::Vector2d(solver_interface->State().x(), solver_interface->State().y()), solver_interface->State().psi(),
                              solver_interface->State().v());
  }
  else if (data_name == "Goal")
  {
    LMPCC_WARN("Guidance: Received a new goal");

    goal_ = data.goal_;
    goal_received_ = true; 
  }
}

void GuidanceConstraints::OnReset(SolverInterface *solver_interface)
{
  ReferencePath::OnReset(solver_interface);

  global_guidance_.Reset();
}

void GuidanceConstraints::ReconfigureCallback(SolverInterface *solver_interface, lmpcc::PredictiveControllerConfig &config,
                                              uint32_t level, bool first_callback)
{
  if (first_callback)
  {
    config.spline_consistency = global_guidance_.GetConfig()->selection_weight_consistency_;
  }
  else
  {
    global_guidance_.GetConfig()->selection_weight_consistency_ = config.spline_consistency;
  }

  global_guidance_.SetReferenceVelocity(config.velocity_reference);
  config_->visualized_guidance_trajectory_nr_ = config.visualize_trajectory_nr;
  config_->highlight_selected_guidance_ = config.highlight_selected;
}

void GuidanceConstraints::ExportData(RosTools::DataSaver &data_saver)
{
  data_saver.AddData("runtime_guidance", global_guidance_.GetLastRuntime());
  for (size_t i = 0; i < planners_.size(); i++) // auto &solver : solvers_)
  {
    auto &planner = planners_[i];
    if (planner.result.success)
      data_saver.AddData("objective_" + std::to_string(i), planner.solver->forces_info_.pobj);
    else
      data_saver.AddData("objective_" + std::to_string(i), -1.);

    if (planner.is_original_planner)
    {
      data_saver.AddData("original_planner_id", planner.id); // To identify which one is the original planner
    }
    auto &vehicle_regions = planner.solver->OptimizedVehiclePredictions(); // Get the ego-vehicle predictions
    for (size_t k = 0; k < vehicle_regions.size(); k++)
      data_saver.AddData("solver" + std::to_string(i) + "_plan" + std::to_string(k), vehicle_regions[k].discs_[0].AsVector2d());

    data_saver.AddData("active_constraints_" + std::to_string(planner.id), planner.guidance_constraints->NumActiveConstraints(planner.solver.get()));
  }

  data_saver.AddData("best_planner_idx", best_planner_index_);
  if (best_planner_index_ != -1)
    data_saver.AddData("gmpcc_objective", planners_[best_planner_index_].solver->forces_info_.pobj);
  else
    data_saver.AddData("gmpcc_objective", -1);

  global_guidance_.ExportData(data_saver);
}