/**
 * @file lmpcc_controller.cpp
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Main controller file
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <lmpcc/lmpcc_controller.h>

#include <generated/modules.h>
#include <ros_tools/profiling.h>

MPCC::MPCC() {}

MPCC::~MPCC()
{
  system_interface_->ActuateBrake(1.5);

  // Save profiling data
  RosTools::Instrumentor::Get().EndSession();
}

// initialize all helper class of predictive control and subscibe joint state and publish controlled joint velocity
bool MPCC::initialize()
{
  if (ros::ok())
  {
    // Initialize the configuration
    config_.reset(new predictive_configuration());

    if (!config_->initialize())
    {
      LMPCC_ERROR("Failed to initialize!");
      return false;
    }

    // Loading Solver and interface (Note the macros in solver_definitions.h)
    solver_interface_.reset(new SolverInterface());
    config_->UpdateWithSolver(solver_interface_.get());

    system_interface_.reset(new INTERFACE_CLASS(nh, this, config_.get(), solver_interface_.get()));

    LMPCC_WARN("Debug mode is ENABLED");

    /******************************** Publishers **********************************************************/
    // computation_pub_ = nh.advertise<std_msgs::Float64>("lmpcc/computation_times", 1);

    /******************************** Visuals **********************************************************/
    collision_space_markers_.reset(
        new RosTools::ROSMarkerPublisher(nh, config_->planned_space_topic_.c_str(), config_->target_frame_, 100)); // 3500)); // was 1800
    current_collision_space_markers_.reset(new RosTools::ROSMarkerPublisher(nh, config_->current_space_topic_.c_str(), config_->target_frame_,
                                                                            solver_interface_->n_discs_)); // 3500)); // was 1800

    // Timer for control
    timer_ = nh.createTimer(ros::Duration(1.0 / config_->clock_frequency_), &MPCC::runNode, this);
    timer_running_ = true;

    prev_x_ = 0.0;
    prev_y_ = 0.0;

    loop_count_ = 1;

    objective_reached_ = false;
    state_received_ = false;

    // Benchmarking
    control_loop_benchmarker_.initialize("Full Control Loop", true);
    optimization_benchmarker_.initialize("Optimization", true);
    module_benchmarkers_.initialize("Module Updates", true);

    // Initialize profiling
    RosTools::Instrumentor::Get().BeginSession("LMPCC");

    /** Create a vehicle object (compute the collision region) const Eigen::Vector2d &pos, const double orientation, int
     * n_discs, double width, double length, double center_offset*/
    vehicle_.reset(new VehicleRegion(Eigen::Vector2d(0., 0.), 0., solver_interface_->n_discs_, config_->vehicle_width_, config_->vehicle_length_,
                                     config_->vehicle_center_of_mass_to_back_));
    // vehicle_.reset(new VehicleRegion(Eigen::Vector2d(0., 0.), 0., *(solver_interface_->area_)));

    if (config_->auto_enable_plan_)
    {
      if (config_->enable_output_)
        enable_output_ = true;

      plan_ = true;

      auto_enable_ = true;
    }
    else
    {
      enable_output_ = false;
      plan_ = false;
      auto_enable_ = false;
    }

    enable_output_ = true;

    // Casting prevents need for conflicting includes in base_model.h (but is not ideal)
    InitializeModules(control_modules_, nh, config_.get(), vehicle_.get());

    // Initialize the dynamic reconfiguration
    LMPCC_INFO("Setting up dynamic_reconfigure server for the parameters");
    first_reconfigure_callback_ = true;
    ros::NodeHandle nh_predictive("predictive_controller");
    reconfigure_server_.reset(new dynamic_reconfigure::Server<lmpcc::PredictiveControllerConfig>(reconfig_mutex_, nh_predictive));
    reconfigure_server_->setCallback(boost::bind(&MPCC::reconfigureCallback, this, _1, _2));

    // See if any module wants to update here
    for (auto &module : control_modules_)
      module->OnDataReceived(solver_interface_.get(), system_interface_->data_, "Init");

    // Get the name of the method based on its modules
    std::string method_name = "";
    for (auto &module : control_modules_)
      module->GetMethodName(method_name);

    config_->recording_name_ += "_" + method_name;
    LMPCC_INFO_ALWAYS("Running method: " << method_name);

    // Setup recording information
    if (config_->record_experiment_)
    {
      experiment_util_.Init(config_.get());
      RosTools::DataSaver &data_saver = experiment_util_.GetDataSaver();

      data_saver.SetAddTimestamp(config_->add_recording_timestamp_);

      data_saver.AddData("disc_radius", vehicle_->discs_[0].radius);
      for (size_t j = 0; j < vehicle_->discs_.size(); j++)
        data_saver.AddData("disc_offset_" + std::to_string(j), vehicle_->discs_[j].offset);

      data_saver.AddData("dt", 1.0 / ((double)config_->clock_frequency_));
    }
    else
    {
      LMPCC_WARN_ALWAYS("Recording is DISABLED");
    }

    controller_status_ = ControllerStatus::WAITING_FOR_DATA;

    LMPCC_SUCCESS_ALWAYS("Controller initialized!");

    return true;
  }
  else
  {
    LMPCC_ERROR("Failed to initialize as ROS Node is shutdown");
    return false;
  }
}

// This function updates with T=1/clock_frequency
void MPCC::runNode(const ros::TimerEvent &event)
{
  if (config_->debug_output_)
    std::cout << std::endl;

  LMPCC_INFO("============ START OF LOOP [" << loop_count_ << "] =======");
  ControlLoop();
  LMPCC_INFO("============ END OF LOOP ===============");

  if (config_->debug_output_)
    std::cout << std::endl;
}

void MPCC::ControlLoop()
{

  PROFILE_AND_LOG(config_->debug_output_, "Control loop");
  RosTools::CallCounter::Get().Loop();

  system_interface_->data_.control_loop_time_ = ros::Time::now();

  if (config_->synchronized_actuation_) // Actuate at the start of the control loop assuming constant computation delay
    system_interface_->ActuateNow();

  unsigned int N_iter;
  int exit_code = 0;

  controller_status_ = ControllerStatus::WAITING_FOR_DATA;

  // Check if each module is ready to execute its control loop
  bool ready_for_control_ = true;
  for (auto &module : control_modules_)
  {
    bool module_is_ready = module->ReadyForControl(solver_interface_.get(), system_interface_->data_);
    ready_for_control_ = ready_for_control_ && module_is_ready;

    if (!module_is_ready)
      LMPCC_WARN_STREAM("module is not ready yet!");
  }

  if (!state_received_)
    LMPCC_WARN("Waiting for the state information from sensors");

  if (system_interface_->data_.dynamic_obstacles_.size() == 0)
  {
    system_interface_->PostProcessObstacles();
    OnObstaclesReceived();
  }
  if (config_->debug_output_)
    system_interface_->data_.Print();

  // The control loop runs if we are planning and all data was received
  if (plan_ && ready_for_control_ && state_received_ && !objective_reached_)
  {
    // Profile the control loop
    control_loop_benchmarker_.start();

    // Initialize the status at success
    controller_status_ = ControllerStatus::SUCCESS;

    /* --------------------------------
    Solver inputs preparation:
        - xinit
        - x0 (through initial_plan)
        - all_parameters
    -------------------------------- */

    vehicle_->SetPosition(Eigen::Vector2d(solver_interface_->State().x(), solver_interface_->State().y()), solver_interface_->State().psi());
    if (config_->synchronized_actuation_)
    {
      solver_interface_->PropagateInitialState(solver_interface_->State()); // Tryout: Propagate the dynamics one step
                                                                            // forward

      for (auto &module : control_modules_)
        module->OnDataReceived(solver_interface_.get(), system_interface_->data_,
                               "State"); // Bit hacky, but we have to update the state in all modules
    }

    solver_interface_->setSolverInitialState();
    solver_interface_->LoadInitialVehiclePredictions();

    // INEQUALITY UPDATE
    {
      PROFILE_AND_LOG(config_->debug_output_, "Module Updates");

      module_benchmarkers_.start();

      for (auto &module : control_modules_)
        module->Update(solver_interface_.get(), system_interface_->data_);

      module_benchmarkers_.stop();
    }

    /* --------------------------------
    Adjust solver inputs:
        - all_parameters:
            - spline parameters
            - objective function weights
            - radius of discs
            - position of discs
            - inequality constraints */

    // INSERTING PARAMETERS
    LMPCC_INFO("Setting Solver Parameters");

    int param_idx; // Parameter idx to write to, must be increased by each parameter loading functionality

    for (N_iter = 0; N_iter < solver_interface_->FORCES_NBAR; N_iter++) // NBAR: N + 2 (opt, opt + constr, opt)
    {
      param_idx = 0;

      // OBJECTIVE (ALL)
      // Load the spline parameters for the reference trajectory
      for (auto &module : control_modules_)
      {
        if (module->type_ == ModuleType::OBJECTIVE)
          module->SetParameters(&(*solver_interface_), system_interface_->data_, N_iter, param_idx);
      }

      // INEQUALITIES (EXCLUDE INITIAL AND FINAL STATE)
      if (N_iter > 0 && N_iter < solver_interface_->FORCES_NBAR - 1)
      {
        // Inequalities
        /** @todo scenario_constraints_->velocity_jackal_ = velocity_; */
        for (auto &module : control_modules_)
        {
          if (module->type_ == ModuleType::CONSTRAINT)
            module->SetParameters(solver_interface_.get(), system_interface_->data_, N_iter, param_idx);
        }
      }
    }
    /* ----------------------------- */

    /* --------------------------------
    Adjust solver inputs:
        - x0: loads the initial plan that may have been projected into the solver to be used as initial guess */
    solver_interface_->loadInitialPlanAsWarmStart();
    /* ----------------------------- */

    // OPTIMIZATION
    {
      PROFILE_AND_LOG(config_->debug_output_, "Optimization");
      optimization_benchmarker_.start();

      /** Idea here: Default optimization methods return exit_code -999
       * If we want to replace the regular optimization with an other optimization, which can happen only once
       * then return a regular exit_code */
      for (auto &module : control_modules_)
      {
        exit_code = module->Optimize(solver_interface_.get());

        if (exit_code != EXIT_CODE_NOT_OPTIMIZED_YET)
        {
          LMPCC_INFO("A module ran the optimization, skipping regular optimization.");
          break;
        }
      }

      if (exit_code == EXIT_CODE_NOT_OPTIMIZED_YET)
      {
        LMPCC_INFO("Running Optimization");
        exit_code = solver_interface_->solve(config_->max_iterations_);
      }

      // solver_interface_->printSolveInfo();

      optimization_benchmarker_.stop();
    }

    // Debug printing
    if (config_->debug_solver_)
      solver_interface_->debugPrintAll();

    // If the solver is infeasible
    if (exit_code != 1)
    {
      // INFEASIBLE
      PROFILE_AND_LOG(config_->debug_output_, "Infeasible");

      controller_status_ = ControllerStatus::FAILURE;
      if (exit_code != previous_exit_code_) // Only print changes
      {
        printInfoAtNoSuccess(exit_code);
      }
      LMPCC_WARN("\tSending fallback commands to the system (not the optimized outputs)");

      // Load an emergency plan into the solver
      system_interface_->DeployEmergencyStrategy(config_->deceleration_at_infeasible_);
      solver_interface_->LoadOptimizedVehiclePredictions(); // This loads the emergency plan for
                                                            // visualization

      // Actuate the emergency plan
      system_interface_->ActuateBrake(config_->deceleration_at_infeasible_);

      loop_count_ = 1;
    }
    else
    {
      // SUCCESSFUL OPTIMIZATION
      PROFILE_AND_LOG(config_->debug_output_, "Success");

      if (exit_code != previous_exit_code_) // Only print changes
        LMPCC_SUCCESS_ALWAYS("Recovered from infeasibility");

      controller_status_ = ControllerStatus::SUCCESS;

      for (auto &module : control_modules_)
        module->OnDataReceived(solver_interface_.get(), system_interface_->data_, "Plan");

      // Load the optimized trajectory into the plan and vehicle predictions for the next control iteration
      solver_interface_->LoadSolution(config_->shift_plan_forward_);

      // Publish the control command
      if (config_->enable_output_)
      {
        LMPCC_INFO("Actuating");
        system_interface_->Actuate();
      }
      else
      {
        LMPCC_INFO("Output is disabled (see enable_output). Braking");

        // deployEmergencyStrategy(config_->deceleration_at_infeasible_);
        system_interface_->ActuateBrake(config_->deceleration_at_infeasible_);
      }

      LMPCC_INFO("Publishing the predicted collision space");
    }

    previous_exit_code_ = exit_code;

    control_loop_benchmarker_.stop();

/* Run the visualization for all modules (can be parallelized)*/
#pragma omp parallel for num_threads(8)
    for (auto &module : control_modules_)
      module->Visualize();

    // Visualize the plan
    publishPlan();

    // Check if we have reached our control objective
    objective_reached_ = true;
    if (!external_objective_reached_)
    {
      for (auto &module : control_modules_)
        objective_reached_ = objective_reached_ && module->ObjectiveReached(solver_interface_.get(), system_interface_->data_);
    }
    if (objective_reached_)
      LMPCC_SUCCESS_ALWAYS("Objective Reached!");

    if (config_->record_experiment_)
      SaveControllerData();

    loop_count_++;
  }
  else
  {
    PROFILE_SCOPE("No Control!");
    // If the controller is not running, maintain braking
    system_interface_->DeployEmergencyStrategy(config_->deceleration_at_infeasible_);
    system_interface_->ActuateBrake(config_->deceleration_at_infeasible_);
    control_loop_benchmarker_.stop();
  }

  // If we do not need to wait before actuating, then send the control commands now
  if (!config_->synchronized_actuation_)
    system_interface_->ActuateNow();

  // Debugging and visuals that are always performed
  if (config_->debug_print_plan_)
    solver_interface_->PrintPlan();
}

void MPCC::SaveControllerData()
{
  LMPCC_INFO("SaveControllerData()");

  if (!state_received_) // Needs to wait until the state is received!
  {
    LMPCC_INFO("Not exporting data: State not yet received.");
    return;
  }

  RosTools::DataSaver &data_saver = experiment_util_.GetDataSaver();
  data_saver.AddData("status", (int)controller_status_);

  data_saver.AddData("runtime_control_loop", control_loop_benchmarker_.getLast());
  data_saver.AddData("runtime_modules", module_benchmarkers_.getLast());
  data_saver.AddData("runtime_optimization", optimization_benchmarker_.getLast());

  double objective = controller_status_ != ControllerStatus::SUCCESS ? -1. : solver_interface_->forces_info_.pobj;
  data_saver.AddData("objective", objective);

  // Export module data if defined
  for (auto &module : control_modules_)
    module->ExportData(data_saver);

  // This will save any data in the solver, external data and vehicle
  experiment_util_.Update(solver_interface_.get(), system_interface_->data_, *vehicle_);
}

void MPCC::OnReset()
{
  LMPCC_INFO("OnReset()");

  // Save the previous state of these variables
  bool plan_prev = plan_;
  bool output_prev = enable_output_;

  // Disable planning / output
  plan_ = false;
  enable_output_ = false;

  // Stop the timer
  if (config_->sync_mode_)
  {
    timer_.stop();
    timer_running_ = false;
  }

  state_received_ = false;
  external_objective_reached_ = false;

  if (config_->record_experiment_)
    experiment_util_.OnTaskCompleted(objective_reached_);

  // Reset solver variables
  solver_interface_->resetSolver();
  for (u_int k = 0; k < solver_interface_->FORCES_N; k++)
    solver_interface_->Plan(k).init(); // Reset the plan!

  // Reset the connected interface
  system_interface_->Reset();

  // Initialize the state of the solver
  solver_interface_->State().init();

  /* Reset all modules */
  for (auto &module : control_modules_)
    module->OnReset(solver_interface_.get());

  velocity_ = 0.;

  loop_count_ = 1;

  // Start the timer again
  if (config_->sync_mode_)
  {
    timer_.start();
    timer_running_ = true;
  }

  // Will trigger a state update before the controller starts
  objective_reached_ = false;

  // Reenable planning and the output
  plan_ = plan_prev;
  enable_output_ = output_prev;

  solver_interface_->resetAtInfeasible();

  controller_status_ = ControllerStatus::RESET;

  LMPCC_SUCCESS_ALWAYS("Reset Completed");
}

void MPCC::OnWeightsReceived()
{
  // Update the configuration (Hack)
  reconfigure_server_->setCallback(boost::bind(&MPCC::reconfigureCallback, this, _1, _2));
}

void MPCC::reconfigureCallback(lmpcc::PredictiveControllerConfig &config, uint32_t level)
{
  LMPCC_INFO("Reconfigure callback!");

  bool is_first_reconfigure_callback = first_reconfigure_callback_;
  if (first_reconfigure_callback_)
  {
    config.enable_debug = config_->debug_output_;
    config.enable_debug_solver = config_->debug_solver_;
    config.enable_output = config_->enable_output_;
    enable_output_ = config_->enable_output_;
    first_reconfigure_callback_ = false;

    // In the first callback, we set the values in the interface
    // Note that we may overwrite unchanged values if we change anything in the config window...
    config.velocity_reference = config_->reference_velocity_;
    system_interface_->reference_velocity_ = config.velocity_reference;
    system_interface_->contouring_weight_ = config.contour;
  }
  else
  {
    config_->debug_output_ = config.enable_debug;
    config_->debug_solver_ = config.enable_debug_solver;
    config_->enable_output_ = config.enable_output;
    enable_output_ = config.enable_output;
  }

  // Use the given velocity reference if it was received
  if (system_interface_->external_velocity_set_)
  {
    config.velocity_reference = system_interface_->reference_velocity_;
    config.contour = system_interface_->contouring_weight_;
  }

  // Update modules if necessary
  for (auto &module : control_modules_)
    module->ReconfigureCallback(solver_interface_.get(), config, level, is_first_reconfigure_callback);

  // reset world
  reset_world_ = config.reset_world;

  if (reset_world_)
  {
    OnReset();
    config.reset_world = false;
  }

  if (config_->record_experiment_ && config.save_data)
  {
    config.save_data = false;
    experiment_util_.ForceDataSave();
  }

  if (auto_enable_)
  {
    LMPCC_WARN("Auto Planning...");
    config.plan = true;

    if (config_->enable_output_)
    {
      config.enable_output = true;
      enable_output_ = true;
    }

    auto_enable_ = false;
  }

  // Take the plan values from the reconfigure
  LMPCC_WARN_ALWAYS("Starting planner...");
  plan_ = config.plan;

  if (plan_ && !reset_world_)
  {
    LMPCC_WARN_ALWAYS("Resetting solver");

    // Reset the solver
    solver_interface_->resetSolver();

    /** @todo Verify that line is not necessary for prius */
    // reference_path_->InitPath();

    objective_reached_ = false;

    if (config_->enable_ext_control_)
    { // in case control is enabled externally: do not run the control loop yet
      timer_.stop();
      timer_running_ = false;
    }
    else
    {
      timer_.start();
      timer_running_ = true;
    }
  }
  else if (!plan_)
  {
    timer_.stop();
    timer_running_ = false;
  }
  LMPCC_SUCCESS("Reset Completed");
}

// On Received functions are executed after the callback is received at the interface
void MPCC::OnObstaclesReceived()
{
  LMPCC_INFO("OnObstaclesReceived()");

  // Update modules
  for (auto &module : control_modules_)
    module->OnDataReceived(solver_interface_.get(), system_interface_->data_, "Dynamic Obstacles");
}

void MPCC::OnOtherDataReceived(std::string &&data_name)
{
  LMPCC_INFO("OnOtherDataReceived()");

  // Update modules
  for (auto &module : control_modules_)
    module->OnDataReceived(solver_interface_.get(), system_interface_->data_, std::forward<std::string>(data_name));
}

void MPCC::OnStateReceived()
{
  LMPCC_INFO("OnStateReceived()");

  // Initialize the spline (and find the closest point to the vehicle)
  for (auto &module : control_modules_)
    module->OnDataReceived(solver_interface_.get(), system_interface_->data_, "State");

  // Check if the state jumped
  bool state_jumped =
      (RosTools::dist(Eigen::Vector2d(solver_interface_->State().x(), solver_interface_->State().y()), Eigen::Vector2d(prev_x_, prev_y_)) > 5.0);
  prev_x_ = solver_interface_->State().x();
  prev_y_ = solver_interface_->State().y();

  // If the first state was received or the state jumped
  if (!state_received_ || state_jumped)
  {
    LMPCC_WARN("Jump in state detected, recomputing path!");

    state_received_ = true;

    // Reset the solver to use the current state
    solver_interface_->resetAtInfeasible();

    // When auto planning, start the planning process (check auto_enable_output)
    if (config_->auto_enable_plan_)
    {
      plan_ = true;

      if (config_->enable_output_)
        enable_output_ = true;
    }
  }
  publishCurrentCollisionSpace();
}

void MPCC::OnWaypointsReceived()
{
  LMPCC_INFO("OnWaypointsReceived()");
  // Initialize the reference path with the received parameters at the interface
  for (auto &module : control_modules_)
    module->OnDataReceived(solver_interface_.get(), system_interface_->data_, "Waypoints");
}

void MPCC::publishPlan()
{
  RosTools::ROSPointMarker &plan_points = collision_space_markers_->getNewPointMarker("CYLINDER");
  RosTools::ROSPointMarker &ellipse = collision_space_markers_->getNewPointMarker("CYLINDER");
  RosTools::ROSLine &line = collision_space_markers_->getNewLine();

  double alpha = 0.3;
  plan_points.setScale(0.2 * config_->scenario_visual_scale_, 0.2 * config_->scenario_visual_scale_, 0.1e-3);
  plan_points.setColor(0., 0., 1.0, 1.0);
  ellipse.setScale(2 * vehicle_->DiscRadius(), 2 * vehicle_->DiscRadius());
  ellipse.setColor(0., 0., 1., alpha);
  line.setScale(0.15 * config_->scenario_visual_scale_, 0.15 * config_->scenario_visual_scale_);
  line.setColor(0., 0., 1.);

  Eigen::Vector2d pose, previous_pose;

  LMPCC_INFO("Visualizing Vehicle Plan");

  auto &vehicle_regions = solver_interface_->OptimizedVehiclePredictions(); // Get the ego-vehicle predictions
  for (size_t k = 0; k < vehicle_regions.size(); k++)
  {
    for (auto &disc : vehicle_regions[k].discs_)
    {
      plan_points.addPointMarker(Eigen::Vector3d(disc.x, disc.y, 0.6e-1));
      ellipse.addPointMarker(Eigen::Vector3d(disc.x, disc.y, 0.5e-3));

      if (k >= 1)
        line.addLine(Eigen::Vector3d(previous_pose(0), previous_pose(1), 0.5e-1), Eigen::Vector3d(disc.x, disc.y, 0.5e-1));

      previous_pose = disc.AsVector2d();
    }
  }

  // Temporary:
  // RosTools::ROSPointMarker &sphere = collision_space_markers_->getNewPointMarker("SPHERE");
  // sphere.setScale(0.3, 0.3, 0.3);
  // sphere.setColorInt(1, 2, 0.5);
  // sphere.addPointMarker(Eigen::Vector3d(og_x_, og_y_, 0.));
  // sphere.setColorInt(0, 2, 0.5);
  // sphere.addPointMarker(Eigen::Vector3d(solver_interface_->State().x(), solver_interface_->State().y(), 0.));

  if (config_->draw_initial_guess_)
    publishInitialPlan();

  collision_space_markers_->publish();
}

void MPCC::publishInitialPlan()
{
  RosTools::ROSPointMarker &plan_points = collision_space_markers_->getNewPointMarker("CYLINDER");
  RosTools::ROSPointMarker &ellipse = collision_space_markers_->getNewPointMarker("CYLINDER");
  RosTools::ROSLine &line = collision_space_markers_->getNewLine();

  double alpha = 0.15;
  plan_points.setScale(0.15 * config_->scenario_visual_scale_, 0.15 * config_->scenario_visual_scale_,
                       0.1e-3); // Points on the lines
  plan_points.setColor(0., 1., 0.0, 1.0);
  ellipse.setScale(2 * vehicle_->DiscRadius(), 2 * vehicle_->DiscRadius()); // Circles
  ellipse.setColor(0., 1., 0., alpha);
  line.setScale(0.12 * config_->scenario_visual_scale_, 0.12 * config_->scenario_visual_scale_); // Lines
  line.setColor(0., 1., 0.);

  Eigen::Vector2d pose, previous_pose;
  for (int d = 0; d < solver_interface_->n_discs_; d++)
  {
    // Draw only the discs that are configured to do so
    if (config_->discs_to_draw_[d] == 0)
      continue;

    LMPCC_INFO("Trajectory Disc: Visualizing Initial Guess");

    // The ego vehicle predictions still contain the initial plan
    auto &vehicle_regions = solver_interface_->InitialVehiclePrediction(); // Get the ego-vehicle predictions
    for (size_t k = 0; k < vehicle_regions.size(); k++)
    {
      for (auto &disc : vehicle_regions[k].discs_)
      {
        plan_points.addPointMarker(Eigen::Vector3d(disc.x, disc.y, 0.6e-1));
        ellipse.addPointMarker(Eigen::Vector3d(disc.x, disc.y, 0.5e-3));

        if (k >= 1)
          line.addLine(Eigen::Vector3d(previous_pose(0), previous_pose(1), 0.5e-1), Eigen::Vector3d(disc.x, disc.y, 0.5e-1));

        previous_pose = disc.AsVector2d();
      }
    }
  }
}

void MPCC::publishCurrentCollisionSpace()
{
  RosTools::ROSPointMarker &ellipse = current_collision_space_markers_->getNewPointMarker("CYLINDER");
  ellipse.setColor(0.0, 0.0, 1.0, 0.8); // BLUE

  // Visualize the current position!
  auto vehicle_regions = solver_interface_->InitialVehiclePrediction();
  if (vehicle_regions.size() == 0)
    return;
  vehicle_regions[0].SetPosition(Eigen::Vector2d(solver_interface_->State().x(), solver_interface_->State().y()), solver_interface_->State().psi());

  for (auto &disc : vehicle_regions[0].discs_)
  {
    ellipse.setScale(disc.radius * 2., disc.radius * 2., 0.01);
    ellipse.addPointMarker(Eigen::Vector3d(disc.x, disc.y, 1e-3));
  }

  current_collision_space_markers_->publish();
}

void MPCC::printInfoAtNoSuccess(int &exit_code) const
{
  switch (exit_code)
  {
  case 0:
    LMPCC_ERROR("Max iterations reached in solver. Check the ForcesPro documentation for more information (exit_code = 0)");
    break;

  case 2:
    LMPCC_ERROR("Solver time-out (exit_code = 2)");
    break;

  case -4:
    LMPCC_ERROR("Wrong number of inequalities input to solver (exit_code = -4)");
    break;

  case -5:
    LMPCC_ERROR("Error occurred during matrix factorization (exit_code = -5)");
    break;

  case -6:
    LMPCC_ERROR("NaN or INF occurred during functions evaluations (exit_code = -6)");
    break;

  case -7:
    LMPCC_ERROR("The solver could not proceed. Check the ForcesPro documentation for more information (exit_code = -7)");
    break;

  case -8:
    LMPCC_ERROR("The internal QP solver could not proceed. Check the ForcesPro for more information (exit_code = -8)");
    break;

  case -10:
    LMPCC_ERROR("NaN or INF occurred during evaluation of functions and derivatives. Check the ForcesPro documentation for "
                "more details (exit_code = -10)");
    break;

  case -11:
    LMPCC_ERROR("Invalid values in problem parameters (exit_code = -11)");
    break;

  case -100:
    LMPCC_ERROR("License error (exit_code = -100)");
    break;

  default:
    LMPCC_ERROR_STREAM("Exit code is not known (exit_code = " << exit_code << ")");
    break;
  }
}