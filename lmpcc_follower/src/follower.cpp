#include <lmpcc_follower/follower.h>

Follower::Follower() { objective_reached_ = false; }

bool Follower::Initialize()
{
  if (ros::ok())
  {

    // Initialize the configuration
    config_.reset(new FollowerConfig());
    if (!config_->Initialize())
      throw std::runtime_error("Follower: Failed to initialize the configuration");

    FOLLOWER_INFO("Reading LMPCC Parameters");
    lmpcc_config_.reset(new predictive_configuration());
    if (!lmpcc_config_->initialize())
      throw std::runtime_error("Follower: Failed to initialize the LMPCC configuration");
    FOLLOWER_SUCCESS("Done reading parameters");
    lmpcc_config_->debug_output_ = false; // Make sure we only print lmpcc related messages when we enable it for the follower

    FOLLOWER_INFO("Initializing Follower");

    // Loading Solver and interface (Note the macros in solver_definitions.h)
    solver_interface_.reset(new SolverInterface());
    // planner_solver_interface_.reset(new SolverInterface());

    planner_horizon_size_ = predictive_configuration::N;

    system_interface_.reset(new SimpleSimInterface(nh_, this, lmpcc_config_.get(), solver_interface_.get()));

    pid_stanley_.reset(new PID_Stanley(nh_)); // Initialize PID-Stanley follower for backup

    timer_follower_ = nh_.createTimer(ros::Duration(1.0 / config_->clock_frequency_), &Follower::Update, this);

    experiment_counter_ = 0;

    ros_markers_reference_path_.reset(new ROSMarkerPublisher(nh_, (lmpcc_config_->reference_path_topic_ + "_follower").c_str(), lmpcc_config_->target_frame_, 30));
    ros_markers_reference_arrows_.reset(new ROSMarkerPublisher(nh_, (lmpcc_config_->reference_arrows_topic_ + "_follower").c_str(), lmpcc_config_->target_frame_, 30));

    planner_sub_ = nh_.subscribe("lmpcc/follower", 1, &Follower::PlannerCallback, this); //, ros::TransportHints().tcpNoDelay());

    FOLLOWER_SUCCESS("Follower Initialized");

    return true;
  }
  else
  {
    FOLLOWER_ERROR("Failed to initialize as ROS Node is shutdown");
    return false;
  }
}

void Follower::PlannerCallback(const lmpcc_msgs::follower &msg)
{
  FOLLOWER_INFO("PlannerCallback()");

  follower_msg_ = msg;
  follower_counter_ = 0;
  planner_horizon_size_ = follower_msg_.trajectory.size();
  time_of_loading_trajectory_ = ros::Time::now();
}

// /** @todo: Convert to msg */
// void Follower::LoadPlannerSolverInfo(SolverInterface *planner_solver_interface, const double planner_optimization_duration)
// {

//   FOLLOWER_INFO("LoadTrajectory()");

//   /** @brief Data
//    * N
//    * State
//    * Plan
//    * Acceleration
//    * */

//   planner_horizon_size_ = planner_solver_interface->FORCES_N;             // Get the horizon length
//   planner_solver_interface_->State() = planner_solver_interface->State(); // Get the state
//   for (unsigned int index_on_horizon = 0; index_on_horizon < planner_horizon_size_; ++index_on_horizon)
//   {
//     planner_solver_interface_->Plan(config_->plan_reference_index_ + index_on_horizon) = planner_solver_interface->Plan(config_->plan_reference_index_ + index_on_horizon); // Copy the horizon plan
//     planner_solver_interface_->a(index_on_horizon) = planner_solver_interface->a(index_on_horizon);                                                                         // Copy the calculated control inputs
//   }

//   follower_counter_ = 0;

//   planner_optimization_duration_ = planner_optimization_duration;

//   time_of_loading_trajectory_ = ros::Time::now();
// }

void Follower::Update(const ros::TimerEvent &event)
{
  PROFILE_FUNCTION();
  FOLLOWER_INFO("Update()");

  CalculatePID_StanleyControls();

  SavePID_StanleyData();

  system_interface_->ActuateFollower(pid_stanley_->GetControlAcceleration(), pid_stanley_->GetControlSteeringRate());

  if (follower_counter_ >= 0)
    follower_counter_ += 1;
}

void Follower::CalculatePID_StanleyControls()
{
  FOLLOWER_INFO("CalculatePID_StanleyControl")
  /** @todo Victor: Is delta or a necessary here? It is 0 in this conversion */
  lmpcc_msgs::follower_state reference_point = SplinePointReference();
  PriusDynamicsState state_reference;
  state_reference.set_x(reference_point.x);
  state_reference.set_y(reference_point.y);
  state_reference.set_psi(reference_point.psi);
  state_reference.set_v(reference_point.v);
  // state_reference.set_a(reference_point.a);

  pid_stanley_->SetReference(state_reference);

  pid_stanley_->SetState(solver_interface_->State()); // Set state to state from solver interface of follower, updating at follower frequency

  pid_stanley_->SetKp_X(config_->Kp_x_);
  pid_stanley_->SetKp_Stanley(config_->Kp_stanley_);
  pid_stanley_->SetKp_V(config_->Kp_v_);
  pid_stanley_->SetKp_Delta(config_->Kp_delta_);
  pid_stanley_->SetKi_V(config_->Ki_v_);
  pid_stanley_->SetKi_Delta(config_->Ki_delta_);
  pid_stanley_->SetKd_V(config_->Kd_v_);
  pid_stanley_->SetKd_Delta(config_->Kd_delta_);

  double time_since_loading_trajectory = ros::Time::now().toSec() - time_of_loading_trajectory_.toSec();
  pid_stanley_->CalculateControls(time_since_loading_trajectory); // If time since loading trajectory exceeds a threshold, emergency braking is enabled.
}

void Follower::SavePID_StanleyData()
{

  // data_saver_.AddData("state_x", solver_interface_->State().x());
  // data_saver_.AddData("state_y", solver_interface_->State().y());
  // data_saver_.AddData("state_psi", solver_interface_->State().psi());
  // data_saver_.AddData("state_v", solver_interface_->State().v());
  // data_saver_.AddData("state_delta", solver_interface_->State().delta());

  // data_saver_.AddData("error_x", pid_stanley_->GetError().x());
  // data_saver_.AddData("error_y", pid_stanley_->GetError().y());
  // data_saver_.AddData("error_psi", pid_stanley_->GetError().psi());
  // data_saver_.AddData("error_v", pid_stanley_->GetError().v());
  // data_saver_.AddData("error_delta", pid_stanley_->GetError().delta());

  // data_saver_.AddData("Control Speed", pid_stanley_->GetControlSpeed());
  // data_saver_.AddData("Control Steering Angle", pid_stanley_->GetControlSteeringAngle());
  // data_saver_.AddData("Control Acceleration", pid_stanley_->GetControlAcceleration());
  // data_saver_.AddData("Control Steering Rate", pid_stanley_->GetControlSteeringRate());
}

lmpcc_msgs::follower_state Follower::SplinePointReference()
{
  FOLLOWER_INFO("SplinePointReference")
  PROFILE_FUNCTION();
  if (follower_counter_ < 0)
  {
    // PriusDynamicsState empty_reference;
    lmpcc_msgs::follower_state empty_reference;
    empty_reference.x = 0.;
    empty_reference.y = 0.;
    empty_reference.psi = 0.;
    empty_reference.v = 0.;
    empty_reference.a = 0.;

    FOLLOWER_WARN_ALWAYS("No trajectory received from LMPCC, returning an empty reference");
    return empty_reference;
  } // Early return if planner info not loaded

  if (follower_counter_ == 0)
  {
    FOLLOWER_INFO("LMPCC trajectory received. Computing reference point.");

    lmpcc_msgs::follower_state &current_state = follower_msg_.current_state;

    std::vector<lmpcc_msgs::follower_state> planner_horizon = {current_state};
    std::vector<double> T = {0.};

    for (unsigned int index_on_horizon = 0; index_on_horizon < follower_msg_.trajectory.size(); ++index_on_horizon)
    {
      planner_horizon.push_back(follower_msg_.trajectory[index_on_horizon]);
      T.push_back((double(index_on_horizon + 1)) * planner_prediction_integrator_stepsize_);
    }

    // Convert trajectory to a vector
    std::vector<double> X, Y, PSI, V, VX, A;
    for (const lmpcc_msgs::follower_state &state : planner_horizon)
    {
      X.push_back(state.x);
      Y.push_back(state.y);
      PSI.push_back(state.psi);
      V.push_back(state.v);
      VX.push_back(state.v * cos(state.psi));
      A.push_back(state.a);
    }

    /** @todo Victor: Check if I reconstructed this correctly with the new interface to lmpcc */
    double global_vx[2]; // Boundary conditions for derivative spline x
    // Your previous version
    // global_vx[0] = planner_horizon.front().v() * cos(planner_horizon.front().psi());
    // global_vx[1] = planner_horizon.back().v() * cos(planner_horizon.back().psi());
    // New version
    global_vx[0] = V.front() * cos(PSI.front());
    global_vx[1] = V.back() * cos(PSI.back());

    double global_vy[2]; // Boundary conditions for derivative spline y
    global_vy[0] = V.front() * sin(PSI.front());
    global_vy[1] = V.back() * sin(PSI.back());

    double global_ax[2]; // Boundary conditions for derivative spline x
    global_ax[0] = A.front() * cos(PSI.front());
    global_ax[1] = A.back() * cos(PSI.back());

    double global_ay[2]; // Boundary conditions for derivative spline y
    global_ay[0] = A.front() * sin(PSI.front());
    global_ay[1] = A.back() * sin(PSI.back());

    spline_x_.reset(new tk::spline);
    spline_x_->set_boundary(tk::spline::first_deriv, global_vx[0],
                            tk::spline::second_deriv, global_ax[1]);
    spline_x_->set_points(T, X);

    spline_y_.reset(new tk::spline);
    spline_y_->set_boundary(tk::spline::first_deriv, global_vy[0],
                            tk::spline::second_deriv, global_ay[1]);
    spline_y_->set_points(T, Y);

    spline_psi_.reset(new tk::spline);
    spline_psi_->set_points(T, PSI);

    time_of_setting_spline = ros::Time::now();

    // Print to termintal to test whether spline fit is correct
    bool print_reference_to_terminal = false;
    if (print_reference_to_terminal)
    {
      const double reference_time_window = 5.;    // Length reference for follower in seconds (future reference trajectory)
      const int amount_of_reference_points = 100; // Points in reference trajectory, size of follower prediction horizon, N.

      double spline_points[amount_of_reference_points];
      double spline_deriv[amount_of_reference_points];

      std::cout << std::endl
                << "Input horizon points x: ";
      for (double x : X)
      {
        std::cout << std::to_string(x) << ", ";
      }
      std::cout << std::endl;

      std::cout << "Spline points: ";
      for (int i = 0; i < amount_of_reference_points; i++)
      {
        spline_points[i] = spline_x_->operator()(double(i) / double(amount_of_reference_points - 1) * reference_time_window);
        std::cout << std::to_string(spline_points[i]) << ", ";
      }
      std::cout << std::endl;

      // std::cout<<std::endl<<"Input horizon deriv x (points v): "<<std::to_string(current_state.v())<<", "<<std::to_string(first_state.v())<<", "<<std::to_string(second_state.v())<<", "<<std::to_string(third_state.v())<<std::endl;
      std::cout << std::endl
                << "Input horizon deriv x (points v) {global_vx[0]: " << global_vx[0] << ", " << global_vx[1]
                << ",global_ax: " << global_ax[0] << ", " << global_ax[1]
                << "}: ";
      for (double vx : VX)
      {
        std::cout << std::to_string(vx) << ", ";
      }
      std::cout << std::endl;

      std::cout << "Spline deriv: ";
      for (int i = 0; i < amount_of_reference_points; i++)
      {

        spline_deriv[i] = spline_x_->deriv(1, double(i) / double(amount_of_reference_points - 1) * reference_time_window);
        std::cout << std::to_string(spline_deriv[i]) << ", ";
      }
      std::cout << std::endl
                << std::endl;

      // std::cout<<"Input horizon points y: "<<std::to_string(current_state.y())<<", "<<std::to_string(first_state.y())<<", "<<std::to_string(second_state.y())<<", "<<std::to_string(third_state.y())<<std::endl;
      // std::cout<<"Spline points: ";

      // for(int i=0; i<amount_of_reference_points; i++) {
      //   spline_points[i] = spline_y_(double(i)/double(amount_of_reference_points-1)*reference_time_window);
      //   std::cout<<std::to_string(spline_points[i])<<", ";
      // }
      // std::cout<<std::endl;

      // std::cout<<"Input horizon points psi: "<<std::to_string(current_state.psi())<<", "<<std::to_string(first_state.psi())<<", "<<std::to_string(second_state.psi())<<", "<<std::to_string(third_state.psi())<<std::endl;
      // std::cout<<"Spline points: ";

      // for(int i=0; i<amount_of_reference_points; i++) {
      //   spline_points[i] = spline_psi_(double(i)/double(amount_of_reference_points-1)*reference_time_window);
      //   std::cout<<std::to_string(spline_points[i])<<", ";
      // }
      // std::cout<<std::endl;
    }
  }

  lmpcc_msgs::follower_state reference_point;
  const double lookahead_point_time = planner_prediction_integrator_stepsize_; // Constant time of point looking ahead of current state to set as reference
  const double time_since_setting_spline = ros::Time::now().toSec() - time_of_setting_spline.toSec();

  const double reference_lookahead_time = lookahead_point_time + time_since_setting_spline + planner_optimization_duration_;

  const double x_reference = spline_x_->operator()(reference_lookahead_time);
  const double y_reference = spline_y_->operator()(reference_lookahead_time);
  const double psi_reference = spline_psi_->operator()(reference_lookahead_time);

  reference_point.x = x_reference;
  reference_point.y = y_reference;
  reference_point.psi = psi_reference;

  data_saver_.AddData("x_reference_spline", x_reference);
  data_saver_.AddData("y_reference_spline", y_reference);
  data_saver_.AddData("psi_reference_spline", psi_reference);

  PublishReferencePath(reference_point);

  return reference_point;
}

void Follower::PublishReferencePath(const lmpcc_msgs::follower_state &reference_point)
{
  const double reference_plotting_time = 5.; // How far into the future the spline should be plotted

  // Plot the spline with the point to be tracked by the follower currently
  ROSLine &reference_line = ros_markers_reference_path_->getNewLine();
  reference_line.setColorInt(7, 10, 0.85);
  reference_line.setScale(0.25, 0.25);
  reference_line.setOrientation(0.0);

  geometry_msgs::Point previous_point;

  for (double t = 0.; t <= reference_plotting_time; t += 0.2)
  {
    // Draw the constraint as a line
    geometry_msgs::Point p;
    p.x = spline_x_->operator()(t);
    p.y = spline_y_->operator()(t);
    p.z = -0.5e-3;

    if (t > 0.)
      reference_line.addLine(previous_point, p);

    previous_point = p;
  }

  ros_markers_reference_path_->publish();

  ROSPointMarker &reference_point_arrow = ros_markers_reference_arrows_->getNewPointMarker("ARROW");
  double arrow_length = reference_point.v * 3. / 5.;
  double orientation = reference_point.psi;
  reference_point_arrow.setScale(arrow_length, arrow_length / 5., arrow_length / 5.);
  reference_point_arrow.setColor(1.0, 0.0, 0.0, 0.75);
  geometry_msgs::Point p;

  // Translate arrow such that their center lies on the spline points (instead of default rear of arrows)
  p.x = reference_point.x - arrow_length * cos(orientation);
  p.y = reference_point.y - arrow_length * sin(orientation);
  reference_point_arrow.setOrientation(orientation);
  reference_point_arrow.addPointMarker(p);

  ros_markers_reference_arrows_->publish();

  ROSPointMarker &reference_orientation_arrows = ros_markers_reference_arrows_->getNewPointMarker("ARROW");
  for (double t = 0.; t <= reference_plotting_time; t += 0.2)
  {
    double vx = spline_x_->deriv(1, t);
    double vy = spline_y_->deriv(1, t);
    double v = sqrt(vx * vx + vy * vy);
    double arrow_length = v * 1. / 5.;
    double orientation = spline_psi_->operator()(t);
    reference_orientation_arrows.setScale(arrow_length, arrow_length / 5., arrow_length / 5.);
    reference_orientation_arrows.setColor(1.0, 0.647, 0.0, 1.0);
    geometry_msgs::Point p;

    // Translate arrow such that their center lies on the spline points (instead of default rear of arrows)
    p.x = spline_x_->operator()(t) - arrow_length * cos(orientation);
    p.y = spline_y_->operator()(t) - arrow_length * sin(orientation);
    p.z = -0.5e-3;
    reference_orientation_arrows.setOrientation(orientation);
    reference_orientation_arrows.addPointMarker(p);
  }

  ros_markers_reference_arrows_->publish();
}

void Follower::ExportData()
{
  FOLLOWER_INFO("ExportData()");
}

void Follower::SaveExportData()
{
  // If recording
  if (lmpcc_config_->record_experiment_)
  {
    // Save every x experiments
    if ((!lmpcc_config_->record_on_time_) && (experiment_counter_ % lmpcc_config_->save_every_x_experiments_ == 0))
      data_saver_.SaveData(lmpcc_config_->recording_name_ + "_follower");

    experiment_counter_++;

    // Save profiling data before we crash the controller
    if (experiment_counter_ >= lmpcc_config_->number_of_experiments_)
      Helpers::Instrumentor::Get().EndSession();

    LMPCC_ASSERT(
        experiment_counter_ < lmpcc_config_->number_of_experiments_ + 1,
        "Completed the given number of experiments. I know it looks like an error, but it is actually a feature ;)"); // Stop when done with the given
                                                                                                                      // number of experiments (+1 for
                                                                                                                      // first simulation)
  }
  else
  {
    experiment_counter_++;
  }
}

void Follower::OnReset()
{
  FOLLOWER_INFO("OnReset()");

  timer_follower_.stop();
  timer_running_ = false;

  SaveExportData();

  system_interface_->Reset();

  // Start the timer again
  timer_follower_.start();
  timer_running_ = true;

  FOLLOWER_SUCCESS_ALWAYS("Reset Completed");
}
