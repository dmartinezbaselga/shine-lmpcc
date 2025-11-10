/**
 * @file lmpcc_configuration.h
 * @author Boaz Floar, Bruno de Brito, Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Configuration class storing all runtime parameters
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef PREDICTIVE_CONFIGURATION_H
#define PREDICTIVE_CONFIGURATION_H

#include <ros/ros.h>

#include <ros_tools/base_configuration.h>

class SolverInterface;
/** Logging Pragmas */
#define LMPCC_INFO(msg)                  \
  if (config_->debug_output_)            \
  {                                      \
    ROS_INFO_STREAM("[LMPCC]: " << msg); \
  }

#define LMPCC_WARN(msg)                  \
  if (config_->debug_output_)            \
  {                                      \
    ROS_WARN_STREAM("[LMPCC]: " << msg); \
  }

#define LMPCC_ERROR(msg) ROS_ERROR_STREAM("[LMPCC]: " << msg)

#define LMPCC_INFO_STREAM(msg)           \
  if (config_->debug_output_)            \
  {                                      \
    ROS_INFO_STREAM("[LMPCC]: " << msg); \
  }

#define LMPCC_WARN_STREAM(msg)           \
  if (config_->debug_output_)            \
  {                                      \
    ROS_WARN_STREAM("[LMPCC]: " << msg); \
  }

#define LMPCC_SUCCESS(msg)                                    \
  if (config_->debug_output_)                                 \
  {                                                           \
    ROS_INFO_STREAM("\033[32m[LMPCC]: " << msg << "\033[0m"); \
  }

#define LMPCC_ERROR_STREAM(msg) ROS_ERROR_STREAM("[LMPCC]: " << msg)

#define LMPCC_INFO_ALWAYS(msg) ROS_INFO_STREAM("[LMPCC]: " << msg)
#define LMPCC_WARN_ALWAYS(msg) ROS_WARN_STREAM("[LMPCC]: " << msg)
#define LMPCC_SUCCESS_ALWAYS(msg) ROS_INFO_STREAM("\033[32m[LMPCC]: " << msg << "\033[0m");

#define LMPCC_INFO_FUNCTION                       \
  if (config_->debug_output_)                     \
  {                                               \
    ROS_INFO_STREAM("[LMPCC]: " << __FUNCTION__); \
  }
#define LMPCC_WARN_FUNCTION                       \
  if (config_->debug_output_)                     \
  {                                               \
    ROS_WARN_STREAM("[LMPCC]: " << __FUNCTION__); \
  }

class predictive_configuration : public RosTools::BaseConfiguration
{
  /**
   *  @brief All neccessary configuration parameter of predictive control repository
   *         Read data from parameter server
   *         Updated old data with new data
   *  Note:  All data member name used like xyz_ and all parameter name is normal like xyz.
   */

public:
  /**
   * @brief predictive_configuration: default constructor of this class
   */
  predictive_configuration();

  /**
   * @brief ~predictive_configuration: defualt distructor of this class
   */
  ~predictive_configuration();

  /**
   * @brief intialize: Read all parameters from the ros parameter server
   * @return TRUE iff all parameter initialize successfully
   */
  bool initialize();

  void UpdateWithSolver(SolverInterface *solver_interface);

  /************ CONFIGURATION VARIABLES **************/
  // DEBUG
  bool debug_output_;
  bool debug_solver_;
  bool scenario_debug_output_;
  double debug_add_speed_;

  static double SOLVER_DT;
  static unsigned int N;

  int N_pedestrians_;

  double solver_timeout_;

  bool initialize_success_;
  bool sync_mode_;
  bool simulation_mode_;
  bool auto_enable_plan_;
  bool enable_output_;

  // Whether a seperate follower will run below MPCC planner
  bool use_follower_;

  // PID follower gains
  int plan_reference_index_;
  double Kp_v_;
  double Kp_delta_;
  double Ki_v_;
  double Ki_delta_;
  double Kd_v_;
  double Kd_delta_;
  double Kp_x_;
  double Kp_stanley_;

  // External interface used to enable MPCC control
  bool enable_ext_control_;

  /** inputs and output topic definition **/
  std::string cmd_, cmd_sim_;
  std::string robot_state_topic_, robot_velocity_topic_, reset_topic_;

  // use for finding kinematic chain and urdf model
  std::string robot_;
  std::string robot_base_link_;
  std::string global_path_frame_; //  End effector of arm
  std::string target_frame_;
  std::string obstacle_prediction_topic_;
  std::string obs_state_topic_;
  std::string steering_state_topic_;
  std::string acceleration_state_topic_;
  std::string waypoint_topic_;
  std::string vref_topic_;
  std::string controller_feedback_;
  std::string free_space_topic_;
  std::string reset_environment_topic_;
  std::string navigation_goal_topic_;
  std::string linear_constraints_topic_;

  // Visualization Topics
  std::string reference_path_topic_;
  std::string reference_arrows_topic_;
  std::string spline_index_topic_;
  std::string current_space_topic_;
  std::string planned_space_topic_;
  std::string planned_trajectory_topic_;
  std::string repulsive_ellipsoids_topic_;
  std::string repulsive_regulations_topic_;
  std::string repulsive_rightofway_topic_;
  std::string obstacles_received_topic_;
  std::string obstacles_predicted_topic_;

  // limiting parameter, use to enforce joint to be in limit
  std::vector<std::string> collision_check_obstacles_;

  // Initialize vectors for reference path points
  std::vector<double> ref_x_;
  std::vector<double> ref_y_;
  std::vector<double> ref_psi_;
  double road_width_right_, road_width_left_;
  bool two_way_road_;
  bool shift_plan_forward_;

  // Steps of delay on the input signal
  int input_delay_;

  // Variables SET IN LMPCC CONTROLLER
  // size_t N_;
  size_t NVAR_;

  // Static obstacles with JPS
  int occ_width_, occ_height_;
  double occ_res_;

  bool static_obstacles_enabled_;
  std::string occupancy_grid_topic_;

  // Scenario related!
  double r_VRU_;
  bool use_json_trajectory;

  int n_halfspaces_;
  bool multithread_scenarios_;
  int sample_size_;
  int batch_count_;
  int removal_count_;
  bool automatically_compute_sample_size_;
  std::vector<int> indices_to_draw_;
  int nr_stages_to_draw_;
  std::vector<int> discs_to_draw_;
  bool draw_all_scenarios_, draw_selected_scenarios_, draw_removed_scenarios_, draw_ellipsoids_, draw_constraints_;
  bool draw_projected_position_, draw_polygon_scenarios_, draw_initial_guess_;
  double scenario_visual_scale_;

  bool debug_boolean1_;
  double debug_double1_;
  int seed_;
  double r_vehicle_;
  bool truncated_;
  double truncated_radius_;
  double manual_noise_;
  bool use_trajectory_sampling_;
  bool enable_termination_;
  double terminate_eq_tol_;
  double n_bar_;
  bool use_real_samples_;
  bool use_1D_projection_;
  bool record_experiment_;
  bool record_on_time_;
  int save_every_x_iterations_;
  int save_every_x_experiments_;
  bool halfspaces_from_spline_;

  bool enable_external_weights_;
  double contouring_weight_;
  double reference_velocity_;
  bool scale_reference_velocity_;

  double weight_acceleration_;
  double weight_angular_velocity_;
  double weight_contour_;
  double weight_lag_;
  double weight_velocity_;

  std::string recording_name_;
  bool add_recording_timestamp_;
  int number_of_experiments_;
  bool enable_clothoid_interpolation_;
  double debug_velocity_;
  bool debug_colors_;
  bool debug_print_plan_;
  double time_out_;
  bool record_video_;
  std::string custom_video_name_;

  double deceleration_at_infeasible_;

  bool build_database_;
  bool synchronized_actuation_;

  // Polygons
  int polygon_checked_constraints_;
  bool propagate_covariance_;
  double polygon_range_;
  double activation_range_;

  // Probabilistic scenario properties
  double risk_;
  double confidence_;
  double received_object_sigma_;

  // Numbers of points for spline and clothoid fitting
  int n_points_clothoid_;
  int n_points_spline_;
  double min_wapoints_distance_;
  double epsilon_;

  // Debug
  double ini_vel_x_;

  // predictive control
  double clock_frequency_;          // hz clock Frequency
  double clock_frequency_follower_; // hz clock Frequency for follower system

  bool enable_projection_;
  bool enable_scenario_removal_;
  int max_iterations_;
  bool save_all_support_;

  int max_obstacles_;
  double vehicle_length_;
  double vehicle_width_;
  double vehicle_center_of_mass_to_back_;

  // GUIDANCE
  bool legacy_icra_mode_;
  bool use_goal_grid_;
  bool use_guidance_as_reference_;
  bool enable_guidance_warmstart_, enable_guidance_constraints_;
  bool guidance_add_original_problem_;
  bool visualize_warmstart_;
  bool highlight_selected_guidance_;
  int visualized_guidance_trajectory_nr_;

private:
};

#endif
