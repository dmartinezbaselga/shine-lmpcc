/**
 * @file lmpcc_controller.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Main controller class
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef LMPCC_LMPCC_H
#define LMPCC_LMPCC_H

// Configuration
#include <lmpcc/lmpcc_configuration.h>

// Helpers / util / types
#include <lmpcc/experiment-util.h>
// #include <lmpcc/types.h>

// // Solver generated
#include <lmpcc_solver/InterfaceDefinition.h>
#include <lmpcc_solver/SolverInclude.h>

// // ros includes
// #include <geometry_msgs/PoseArray.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <pluginlib/class_loader.h>
// #include <std_msgs/Bool.h>
// #include <std_msgs/Float32.h>
// #include <std_msgs/Float64.h>
// #include <std_msgs/Float64MultiArray.h>

// // yaml parsing
// #include <yaml-cpp/yaml.h>

// Dynamic Reconfigure server
#include <dynamic_reconfigure/server.h>
#include <lmpcc/PredictiveControllerConfig.h>

// // actions, srvs, msgs
// #include <actionlib/client/simple_action_client.h>
// #include <actionlib/server/simple_action_server.h>
// #include <nav_msgs/Odometry.h>
// #include <nav_msgs/Path.h>

// // lmpcc messages
// #include <lmpcc_msgs/Control.h>
// #include <lmpcc_msgs/halfspace.h>
// #include <lmpcc_msgs/halfspace_array.h>
// #include <lmpcc_msgs/lmpcc_obstacle.h>
// #include <lmpcc_msgs/lmpcc_obstacle_array.h>

// // Joint states
// #include <sensor_msgs/JointState.h>

// Visuals
// #include <ros_tools/data_saver.h>
#include <ros_tools/helpers.h>
#include <ros_tools/ros_visuals.h>

#include <ros/ros.h>
// // reset msgs
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/PoseWithCovariance.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <std_srvs/Empty.h>

// #include <algorithm>
// #include <boost/filesystem.hpp>
// #include <boost/scoped_ptr.hpp>
// #include <boost/shared_ptr.hpp>
// #include <boost/thread/mutex.hpp>
// #include <cmath>
// #include <fstream>
// #include <iomanip>
// #include <iostream>
// #include <limits>
// #include <map>
// #include <math.h>
#include <memory>
// #include <numeric>
// #include <ros/package.h>
// #include <ros/ros.h>
// #include <sstream>
// #include <stdexcept>
// #include <string>
// #include <tf/tf.h>
// #include <tf/transform_listener.h>
// #include <thread>
#include <vector>

/**
 * @brief Enumerator used for tracking the control status
 */
enum class ControllerStatus
{
  RESET = 0,
  WAITING_FOR_DATA = 1,
  SUCCESS = 2,
  FAILURE = 3,
  DONE = 4
};

class Interface;

/**
 * @brief Controller class for Model Predictive Contouring Control.
 */
class MPCC : public Controller
{
public:
  MPCC();
  virtual bool initialize();

  ~MPCC();

public:
  // Dynamic reconfigure server (for updating parameters online)
  boost::shared_ptr<dynamic_reconfigure::Server<lmpcc::PredictiveControllerConfig>> reconfigure_server_;
  boost::recursive_mutex reconfig_mutex_;
  void reconfigureCallback(lmpcc::PredictiveControllerConfig &config, uint32_t level); 

  virtual void OnReset() override;             /**   * @brief Handles a reset of the controller   */
  void OnObstaclesReceived() override; /**   @brief Called when obstacle data is received, forwards data if necessary   */
  void OnStateReceived() override;     /**   @brief Called when the state is received, checks for jumps in the state to reinitialize the reference path   */
  void OnWaypointsReceived() override; /**   @brief Called when waypoints are received, reinitalizes the reference path   */
  void OnWeightsReceived() override;   /**   @brief Called when external weights are received, updates the reconfigure window and local variables   */
  void OnOtherDataReceived(std::string &&data_name) override;

public:
  std::unique_ptr<SolverInterface> solver_interface_; /** @brief Interface with the Force Pro optimization solver */
  /* Interface with the system (i.e., simulation environment or real robot). All sensor and actuator data should flow through this interface! */
  std::unique_ptr<INTERFACE_CLASS> system_interface_;

  // Control modules (i.e., the inequalities and objective)
  std::vector<std::unique_ptr<ControllerModule>> control_modules_;

  ControllerStatus controller_status_; /* Controller status */

  double velocity_; /** @todo: Integrate properly (temporary variable) */

  ros::Timer timer_;
  bool timer_running_;

  // ros::Publisher computation_pub_; // Computation times

protected:
  boost::shared_ptr<predictive_configuration> config_; /** @brief parameters */

  /** Controller options */
  bool enable_output_;              /* Actuate if true */
  bool reset_world_;                /* Resets the environment or robot */
  bool plan_;                       /* Controller is planning if true */
  bool auto_enable_;                /* Start planning on launch if enabled */
  bool first_reconfigure_callback_; /* Used to update the reconfigure window */
  unsigned int loop_count_;         /* Keeps track of amount of control loop calls during runtime */

  int previous_exit_code_ = -1e3;

  std::unique_ptr<VehicleRegion> vehicle_;

  ros::NodeHandle nh;

  // Lightweight classes for profiling the controller
  RosTools::Benchmarker optimization_benchmarker_, control_loop_benchmarker_, module_benchmarkers_;

  ExperimentUtility experiment_util_; /* Utility for experiment monitoring, data saving and autotuning */

  /** The classes for publishing visuals @see ros_visuals.h */
  std::unique_ptr<RosTools::ROSMarkerPublisher> collision_space_markers_;
  std::unique_ptr<RosTools::ROSMarkerPublisher> current_collision_space_markers_;

  /** Checks if a first state reading was received before starting the controller */
  bool state_received_;
  double prev_x_, prev_y_; // Last x and y received

  double initial_velocity_reference_;

  // Responsible for
  // - Updating real-time data
  // - Constructing constraints and inserting solver parameters
  // - Solving the optimization
  // - Fault handling, and
  // - Actuation
  virtual void ControlLoop();                         // Main control function, executed with the control frequency
  virtual void runNode(const ros::TimerEvent &event); // Runs <ControlLoop>"()"

  void SaveControllerData(); // Store control data for saving later

  void printInfoAtNoSuccess(int &exit_code) const; // Print the error and exit code in case the optimization is not successful
  void publishCurrentCollisionSpace(void);         // Publish the collision space of the robot for the received state
  void publishPlan(void);                          // Visualize the robot collision space along the plan.
  void publishInitialPlan(void);                   // Visualize the initial guess before optimization
};

#endif
