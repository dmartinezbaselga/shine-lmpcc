
#include <lmpcc/lmpcc_configuration.h>

#include <lmpcc_solver/SolverInclude.h>

double predictive_configuration::SOLVER_DT = 0.1; // Static variable initialization
unsigned int predictive_configuration::N = 40;    // Static variable initialization

predictive_configuration::predictive_configuration() { initialize_success_ = false; }

predictive_configuration::~predictive_configuration() {}

bool predictive_configuration::initialize()
{
  ros::NodeHandle nh_config;
  ros::NodeHandle nh;

  // High-level settings
  retrieveParameter(nh, "simulation_mode", simulation_mode_);
  retrieveParameter(nh, "sync_mode", sync_mode_);
  retrieveParameter(nh, "auto_enable_plan", auto_enable_plan_);
  retrieveParameter(nh, "clock_frequency", clock_frequency_);
  retrieveParameter(nh, "enable_ext_control", enable_ext_control_, false);

  retrieveParameter(nh, "shift_plan_forward", shift_plan_forward_, false);
  retrieveParameter(nh, "solver_timeout", solver_timeout_, 0.);
  if (solver_timeout_ == 0.)
    solver_timeout_ = 100000; // Set to 100 seconds

  // Frame parameters
  retrieveParameter(nh, "robot_base_link", robot_base_link_);
  retrieveParameter(nh, "target_frame", target_frame_);

  // Path parameters
  retrieveParameter(nh, "global_path/x", ref_x_);
  retrieveParameter(nh, "global_path/y", ref_y_);
  retrieveParameter(nh, "global_path/theta", ref_psi_);

  retrieveParameter(nh, "global_path/epsilon", epsilon_);
  retrieveParameter(nh, "global_path/n_points_clothoid", n_points_clothoid_);
  retrieveParameter(nh, "global_path/n_points_spline", n_points_spline_);
  retrieveParameter(nh, "global_path/min_waypoints_distance", min_wapoints_distance_);
  retrieveParameter(nh, "global_path/frame", global_path_frame_);

  retrieveParameter(nh, "recording/enable_recording", record_experiment_, false);
  retrieveParameter(nh, "recording/experiment", recording_name_, std::string("no_name"));

  std::string project_name, prepend_name;
  retrieveParameter(nh, "recording/project_name", project_name, std::string(""));
  retrieveParameter(nh, "recording/prepend_name", prepend_name, std::string(""));

  std::string segment; // If the name is a file name, remove the file extension
  std::getline(std::stringstream(recording_name_), segment, '.');
  std::replace(segment.begin(), segment.end(), '/', '-'); // replace all '/' with '-'
  recording_name_ = segment;
  if (project_name != "")
    recording_name_ = project_name + "/" + prepend_name + recording_name_; // Put it in the project folder

  retrieveParameter(nh, "recording/number_of_experiments", number_of_experiments_, 100);

  // Record on time by default or if record on experiment was false
  retrieveParameter(nh, "recording/per_time/enable", record_on_time_, true);
  bool record_on_experiment;
  retrieveParameter(nh, "recording/per_experiment/enable", record_on_experiment, false);
  record_on_time_ = record_on_time_ || (!record_on_experiment);

  retrieveParameter(nh, "recording/per_time/every_x_iterations", save_every_x_iterations_, 200);
  retrieveParameter(nh, "recording/per_experiment/every_x_experiments", save_every_x_experiments_, -1);
  if (save_every_x_experiments_ == -1)
    save_every_x_experiments_ = number_of_experiments_;

  retrieveParameter(nh, "recording/add_timestamp", add_recording_timestamp_, false);

  retrieveParameter(nh, "recording/time_out", time_out_, 0.0);
  if (time_out_ == 0.0) // Disable
    time_out_ = 1.0e9;

  // Sample parameters
  retrieveParameter(nh, "scenarios/safe_sampling/compute_automatically", automatically_compute_sample_size_);
  retrieveParameter(nh, "scenarios/safe_sampling/risk", risk_);
  retrieveParameter(nh, "scenarios/safe_sampling/confidence", confidence_);
  retrieveParameter(nh, "scenarios/safe_sampling/removal_count", removal_count_);
  retrieveParameter(nh, "scenarios/safe_sampling/sample_size", sample_size_);

  retrieveParameter(nh, "deceleration_at_infeasible", deceleration_at_infeasible_, 2.0);
  retrieveParameter(nh, "debug_velocity", debug_velocity_, 0.0);
  retrieveParameter(nh, "debug_colors", debug_colors_, false);
  retrieveParameter(nh, "debug_print_plan", debug_print_plan_, false);
  retrieveParameter(nh, "enable_output", enable_output_);
  retrieveParameter(nh, "use_follower", use_follower_);

  retrieveParameter(nh, "scenarios/trajectory/sqp_iterations", max_iterations_);
  retrieveParameter(nh, "scenarios/trajectory/save_all_support", save_all_support_);
  retrieveParameter(nh, "scenarios/trajectory/n_bar", n_bar_);
  retrieveParameter(nh, "scenarios/trajectory/terminate_equality_tolerance", terminate_eq_tol_);
  retrieveParameter(nh, "scenarios/trajectory/enable_termination", enable_termination_);

  retrieveParameter(nh, "path/enable_clothoid_interpolation", enable_clothoid_interpolation_);
  retrieveParameter(nh, "path/halfspaces_from_spline", halfspaces_from_spline_);

  /////////////// TOPICS /////////////////////////
  // Publish
  retrieveParameter(nh, "publish/control_command", cmd_);
  retrieveParameter(nh, "publish/reset", reset_topic_);
  retrieveParameter(nh, "publish/feedback", controller_feedback_, std::string());
  retrieveParameter(nh, "publish/reset_environment", reset_environment_topic_, std::string());
  retrieveParameter(nh, "publish/navigation_goal", navigation_goal_topic_, std::string());

  // Subscribe
  retrieveParameter(nh, "subscribe/state", robot_state_topic_);
  retrieveParameter(nh, "subscribe/velocity", robot_velocity_topic_,
                    std::string()); // can be left open if state is read
  retrieveParameter(nh, "subscribe/obstacles", obs_state_topic_);
  retrieveParameter(nh, "subscribe/obstacle_predictions", obstacle_prediction_topic_);
  retrieveParameter(nh, "subscribe/waypoints", waypoint_topic_);
  retrieveParameter(nh, "subscribe/steering_angle", steering_state_topic_);
  retrieveParameter(nh, "subscribe/acceleration", acceleration_state_topic_);
  retrieveParameter(nh, "subscribe/velocity_reference", vref_topic_);
  retrieveParameter(nh, "subscribe/occupancy_grid", occupancy_grid_topic_);
  retrieveParameter(nh, "subscribe/linear_constraints", linear_constraints_topic_);

  // Visualization
  retrieveParameter(nh, "visualization/reference_path", reference_path_topic_);
  retrieveParameter(nh, "visualization/reference_arrows", reference_arrows_topic_);
  retrieveParameter(nh, "visualization/spline_index", spline_index_topic_);
  retrieveParameter(nh, "visualization/current_collision_space", current_space_topic_);
  retrieveParameter(nh, "visualization/planned_collision_space", planned_space_topic_);
  retrieveParameter(nh, "visualization/planned_trajectory", planned_trajectory_topic_);
  retrieveParameter(nh, "visualization/free_space", free_space_topic_);
  retrieveParameter(nh, "visualization/obstacles_received", obstacles_received_topic_);
  retrieveParameter(nh, "visualization/obstacles_predicted", obstacles_predicted_topic_);

  retrieveParameter(nh, "road/width_right", road_width_right_);
  retrieveParameter(nh, "road/width_left", road_width_left_);
  retrieveParameter(nh, "road/two_way", two_way_road_);

  retrieveParameter(nh, "enable_external_weights", enable_external_weights_, false);
  retrieveParameter(nh, "simulation/weights/contouring", contouring_weight_, 0.);
  retrieveParameter(nh, "reference_velocity", reference_velocity_, 0.);
  retrieveParameter(nh, "scenarios/scale_reference_velocity", scale_reference_velocity_, false);

  retrieveParameter(nh, "synchronized_actuation", synchronized_actuation_, false);

  // retrieveParameter(nh, "obstacles/radius", r_VRU_);
  retrieveParameter(nh, "obstacles/safe_radius", r_VRU_);
  retrieveParameter(nh, "obstacles/max_obstacles", max_obstacles_);
  if (max_obstacles_ == -1)
  {
    retrieveParameter(nh, "carla/scenario/method/max_obstacles", max_obstacles_);
  }

  retrieveParameter(nh, "scenarios/marginal/propagate_covariance", propagate_covariance_);
  retrieveParameter(nh, "scenarios/manual_noise", manual_noise_);

  retrieveParameter(nh, "robot/length", vehicle_length_);
  retrieveParameter(nh, "robot/width", vehicle_width_);
  retrieveParameter(nh, "robot/com_to_back", vehicle_center_of_mass_to_back_);

  retrieveParameter(nh, "debug_output", debug_output_);
  retrieveParameter(nh, "debug_solver", debug_solver_);
  retrieveParameter(nh, "scenarios/debug_output", scenario_debug_output_, false);

  retrieveParameter(nh, "scenarios/use_trajectory_sampling", use_trajectory_sampling_);
  retrieveParameter(nh, "scenarios/trajectory/use_real_samples", use_real_samples_, false);
  retrieveParameter(nh, "scenarios/trajectory/use_json_trajectory", use_json_trajectory, false);
  retrieveParameter(nh, "scenarios/trajectory/enable_projection", enable_projection_);
  retrieveParameter(nh, "scenarios/trajectory/enable_scenario_removal", enable_scenario_removal_);
  retrieveParameter(nh, "scenarios/trajectory/use_1D_projection", use_1D_projection_);

  retrieveParameter(nh, "scenarios/marginal/database/truncated", truncated_);
  retrieveParameter(nh, "scenarios/marginal/database/truncated_radius", truncated_radius_);
  retrieveParameter(nh, "scenarios/marginal/database/build_database", build_database_);
  retrieveParameter(nh, "scenarios/marginal/database/size", batch_count_);

  retrieveParameter(nh, "debug/boolean1", debug_boolean1_, false);
  retrieveParameter(nh, "debug/double1", debug_double1_, 0.);

  /* Scenario related parameters */

  retrieveParameter(nh, "scenarios/polygon/range", polygon_range_);
  retrieveParameter(nh, "scenarios/polygon/activation_range", activation_range_);

  retrieveParameter(nh, "scenarios/static_obstacles/enable", static_obstacles_enabled_);
  retrieveParameter(nh, "scenarios/static_obstacles/n_halfspaces", n_halfspaces_);

  retrieveParameter(nh, "scenarios/multithread", multithread_scenarios_);

  retrieveParameter(nh, "scenarios/visualisation/indices_to_draw", indices_to_draw_, std::vector<int>{});
  retrieveParameter(nh, "scenarios/visualisation/nr_stages_to_draw", nr_stages_to_draw_, 0);

  retrieveParameter(nh, "scenarios/visualisation/discs_to_draw", discs_to_draw_);
  retrieveParameter(nh, "scenarios/visualisation/all_scenarios", draw_all_scenarios_);
  retrieveParameter(nh, "scenarios/visualisation/support_scenarios", draw_selected_scenarios_);
  retrieveParameter(nh, "scenarios/visualisation/polygon_scenarios", draw_polygon_scenarios_);
  retrieveParameter(nh, "scenarios/visualisation/removed_scenarios", draw_removed_scenarios_);
  retrieveParameter(nh, "scenarios/visualisation/initial_guess", draw_initial_guess_);
  retrieveParameter(nh, "scenarios/visualisation/ellipsoids", draw_ellipsoids_);
  retrieveParameter(nh, "scenarios/visualisation/constraints", draw_constraints_);
  retrieveParameter(nh, "scenarios/visualisation/scale", scenario_visual_scale_);

  /* GUIDANCE */
  retrieveParameter(nh, "guidance/constraints/add_original_problem", guidance_add_original_problem_);
  retrieveParameter(nh, "guidance/constraints/enable", enable_guidance_constraints_);
  retrieveParameter(nh, "guidance/warmstart/enable", enable_guidance_warmstart_);
  retrieveParameter(nh, "guidance/warmstart/visualize", visualize_warmstart_);
  retrieveParameter(nh, "guidance/visualization/highlight_selected", highlight_selected_guidance_, true);
  retrieveParameter(nh, "guidance/visualization/only_trajectory_nr", visualized_guidance_trajectory_nr_, -1);
  retrieveParameter(nh, "guidance/use_as_reference", use_guidance_as_reference_);
  retrieveParameter(nh, "guidance/legacy_icra_mode", legacy_icra_mode_);
  retrieveParameter(nh, "guidance/use_goal_grid", use_goal_grid_, false);

  // The following parameters have default settings if not on the configuration
  nh.param("costmap/costmap/width", occ_width_, int(40));
  nh.param("costmap/costmap/height", occ_height_, int(40));
  nh.param("costmap/costmap/resolution", occ_res_, double(0.5));

  nh.param("scenarios/marginal/seed", seed_, -1);

  retrieveParameter(nh, "scenarios/marginal/checked_constraints", polygon_checked_constraints_);

  initialize_success_ = true;

  ROS_WARN("[LMPCC]: Parameter Configuration Initialized");
  return true;
}

void predictive_configuration::UpdateWithSolver(SolverInterface *solver_interface)
{
  predictive_configuration::SOLVER_DT = solver_interface->DT;
  predictive_configuration::N = solver_interface->FORCES_N;
  N_pedestrians_ = predictive_configuration::N;

  solver_interface->setTimeout(time_out_ / 1000.);

  // Divide the number of stages to draw over the horizon
  if (nr_stages_to_draw_ > 0)
  {
    indices_to_draw_.clear();

    if (nr_stages_to_draw_ == 1)
      indices_to_draw_.push_back(0);
    else if (nr_stages_to_draw_ == 2)
    {
      indices_to_draw_.push_back(0);
      indices_to_draw_.push_back(solver_interface->FORCES_N);
    }
    else
    {
      indices_to_draw_.push_back(0);

      double step = (double)(solver_interface->FORCES_N - 1) / (double)(nr_stages_to_draw_ - 1);
      double cur = 0.;
      for (int i = 0; i < nr_stages_to_draw_ - 2; i++)
      {
        cur += step;
        indices_to_draw_.push_back(std::ceil(cur));
      }

      indices_to_draw_.push_back(solver_interface->FORCES_N - 1);
    }
  }
}
