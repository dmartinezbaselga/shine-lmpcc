/**
 * @file types.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Defines basic classes for use in LMPCC
 * @version 0.1
 * @date 2022-05-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __TYPES_H__
#define __TYPES_H__

#include <lmpcc/dynamic_obstacle.h>

#include <ros_tools/types.h>
#include <ros_tools/data_saver.h>
#include <ros_tools/profiling.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <Eigen/Dense>

#include <string>
#include <vector>
#include <unordered_map>

// To distinguish custom from regular optimization loops.
#define EXIT_CODE_NOT_OPTIMIZED_YET -999

// Forward Declarations
class predictive_configuration;
class SolverInterface;
namespace lmpcc
{
  class PredictiveControllerConfig;
}

class Controller
{
public:
  Controller(){};
  virtual void OnOtherDataReceived(std::string &&data_name) = 0;
  virtual void OnObstaclesReceived() = 0;
  virtual void OnStateReceived() = 0;
  virtual void OnWaypointsReceived() = 0;
  virtual void OnWeightsReceived() = 0;
  virtual void OnReset() = 0;
  virtual void SetExternalObjectiveReached() { external_objective_reached_ = true; }
  virtual void RunOnDemand() { }

  bool objective_reached_ = false;
  bool external_objective_reached_ = false;
  double velocity_ = 0.;
};

// MINOR TYPES
struct Path
{
  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> psi_;

  /**
   * @brief Construct a new Path object
   *
   * @param length Size to allocate (reserve)
   */
  Path(int length = 10);

  void AddPose(const geometry_msgs::Pose &pose);
  void Clear();

  friend std::ostream &operator<<(std::ostream &out, const Path &path)
  {

    out << "Path:\n";
    for (size_t i = 0; i < path.x_.size(); i++)
    {
      out << "(" << path.x_[i] << ", " << path.y_[i] << ", " << path.psi_[i] << ")" << std::endl;
    }
    return out;
  }
};

/** @brief Implementation of a cubic spline that can be intialized with a,b,c,d and a distance */
class CubicSpline
{
public:
  std::vector<double> a_, b_, c_, d_;
  std::vector<double> knot_distances_;

  CubicSpline();

  // Add a spline
  void AddSpline(double a, double b, double c, double d, double L);

  // Add a spline
  void AddSpline(const Eigen::Vector4d &abcd, double L);

  // Evaluate the spline at distance s
  double operator()(double s) const;

  /** @brief Add interface for retrieving a, b, c, d */
  void GetParameters(int index, double &a, double &b, double &c, double &d);

  double GetSplineStart(int index);
  double GetSplineEnd(int index);

  friend std::ostream &operator<<(std::ostream &stream, const CubicSpline &spline)

  {
    stream << "==== CubicSpline ====\n";
    for (size_t i = 0; i < spline.a_.size(); i++)
    {
      stream << "s" << i << " = [" << spline.knot_distances_[i] << ", " << spline.knot_distances_[i + 1] << "]\n"
             << "coef" << i << " = [" << spline.a_[i] << ", " << spline.b_[i] << ", " << spline.c_[i] << ", " << spline.d_[i] << "]\n";
    }

    return stream;
  }
};

/**
 * @brief A wrapper class to track if data was used yet and if new data has been received
 *
 * @tparam T The variable type
 */
template <class T>
class Tracked
{
public:
  Tracked() { data_is_new_ = false; };

public:
  void Set(const T &new_value)
  {
    value_ = new_value;
    data_is_new_ = true;
  }

  /**
   * @brief Get the data
   *
   * @param keep_data_flag Set to true if the operation should not change the used status of the data to false
   * @return T&
   */
  T &Get(bool keep_data_flag = false)
  {
    data_is_new_ = false & keep_data_flag;
    return value_;
  };

  bool DataIsNew() const { return data_is_new_; }

private:
  T value_;
  bool data_is_new_;
};

/**
 * @brief A wrapper for std::vector that tracks if new data was inserted. To be used for tracking the arrival of real-time data.
 *
 * @tparam T
 */
template <class T>
class TrackedVector : public std::vector<T>
{
public:
  TrackedVector() : std::vector<T>() { data_is_new_ = false; };

  /**
   * @brief Copy constructor from a vector
   *
   * @param other a std::vector
   */
  TrackedVector(const std::vector<T> &other) : std::vector<T>(other) { data_is_new_ = true; };

public:
  /**
   * @brief A wrapper for emplace_back, sets new data arrived to true
   * @todo Set data_is_new_ to false when data was used
   *
   * @tparam Args
   * @param args
   */
  template <typename... Args>
  void emplace_back(Args &&...args)
  {
    std::vector<T>::emplace_back(std::forward<Args>(args)...);
    data_is_new_ = true;
  }

  /**
   * @brief Wrapper for push_back that sets new data arrived to true
   *
   * @param new_value
   */
  void push_back(const T &new_value)
  {
    std::vector<T>::push_back(new_value);
    data_is_new_ = true;
  }

  /**
   * @brief True if new data was received and not yet used
   *
   * @return true
   * @return false
   */
  bool DataIsNew() const { return data_is_new_; }

private:
  bool data_is_new_;
};

// MAJOR TYPES

/**
 * @brief Stores relevant real-time data in one place
 *
 */
class RealTimeData
{

public:
  RealTimeData(int n_dynamic_obstacles = 0, int n_static_obstacles = 0);

  virtual ~RealTimeData();

  void Print();

public:
  // Obstacles
  TrackedVector<DynamicObstacle> dynamic_obstacles_;           // Dynamic -> /** @Todo: Make a type for this? */
  TrackedVector<std::vector<RosTools::Halfspace>> halfspaces_; // Static (N x Nh)
  double intrusion_;                                           // Detected collisions externally

  ros::Time state_received_time_, obstacles_received_time_, control_loop_time_;

  // Path
  Tracked<Path> path_;

  // Goal Position
  Eigen::Vector2d goal_;

  // States that are also actuated
  double velocity_;
  std::vector<double> reference_velocity_;

  struct Partition
  {
    int id;
    double velocity;
  };
  std::unordered_map<int, Partition> obstacle_partitions_;

protected:
};

/** @todo Simpler monitor class */
class Monitor
{
public:
  Monitor(ros::NodeHandle &nh);

public:
  /** @brief Mark that the callback with name "name" is expected to be called in each iteration. If only once is enough, set persistent to true */
  void MarkExpected(const std::string &&name, bool persistent = false);

  void MarkReceived(const std::string &&name);

  // Print the status of all the signals and reset
  void PrintStatus();

private:
  std::map<std::string, bool> received_;
  std::map<std::string, bool> persistent_;

  ros::Publisher text_publisher_;
};

enum class ModuleType
{
  OBJECTIVE = 0,
  CONSTRAINT,
  UNDEFINED
};

/**
 * @brief Purely virtual module of the controller to compute inequalities, objective terms or anything else.
 * The idea is that modules are defined in the solver and seemingly integrate with the c++ code without having to adapt parameters on either side.
 * This should make the process of stacking different MPC contributions more flexible.
 *
 * @todo Define an "external module" that works with services to be able to connect any other node with this planner, e.g., Python based
 * implementations.
 */
class ControllerModule
{
public:
  /**
   * @brief Construct a new Controller Module object. Note that controller module initialization happens in the solver class itself based on the
   * python code.
   */
  ControllerModule(ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle);
  virtual ~ControllerModule(){};

public:
  /** ====== MANDATORY FUNCTIONS (purely virtual) ==========*/

  /**
   * @brief Update the module (any computations that need to happen before setting solver parameters)
   *
   * @param solver_interface
   */
  virtual void Update(SolverInterface *solver_interface, RealTimeData &data) = 0;

  /**
   * @brief Insert computed parameters for the solver
   *
   * @param solver_interface
   */
  virtual void SetParameters(SolverInterface *solver_interface, const RealTimeData &data, int N_iter, int &param_idx) = 0;

  /**
   * @brief Visualize the computations in this module
   *
   */
  virtual void Visualize() = 0;
  /* ======================================================== */

  /** ====== OPTIONAL FUNCTIONS ==========*/

  /**
   * @brief Check if this module is ready for control
   *
   * @return true If this module can execute the necessary computations with the current data
   * @return false Otherwise
   */
  virtual bool ReadyForControl(SolverInterface *solver_interface, const RealTimeData &data) { return true; }; // Default: true

  /**
   * @brief Check if the objective of this module was reached
   *
   * @param solver_interface
   * @param data
   * @return true If the objective was reached
   */
  virtual bool ObjectiveReached(SolverInterface *solver_interface, const RealTimeData &data) { return true; }; // Default: true

  /**
   * @brief Function used to update any class members when new data is received
   *
   * @param solver_interface The solver data. Necessary because the state is defined in this class
   * @param data All real-time data
   * @param data_name The name of the data that was updated (to decide if anything needs to be updated)
   */
  virtual void OnDataReceived(SolverInterface *solver_interface, RealTimeData &data, std::string &&data_name){};

  /**
   * @brief Reset any members if necessary
   *
   * @param solver_interface
   */
  virtual void OnReset(SolverInterface *solver_interface){};

  /**
   * @brief Override to define a custom optimization loop. Note that there can only be ONE customized optimization.
   *
   * @return int exit_code of the solver, return any exit_code other than "EXIT_CODE_NOT_OPTIMIZED_YET" to define this as a custom optimization
   */
  virtual int Optimize(SolverInterface *solver_interface) { return EXIT_CODE_NOT_OPTIMIZED_YET; }; // Default: no custom optimization

  /**
   * @brief Read parameters from the dynamic reconfiguration
   *
   * @param config The parameters from the configuration
   */
  virtual void ReconfigureCallback(SolverInterface *solver_interface, lmpcc::PredictiveControllerConfig &config, uint32_t level, bool first_callback){};

  /**
   * @brief Export runtime data
   *
   * @param data_saver the data_saver object to add the data to
   */
  virtual void ExportData(RosTools::DataSaver &data_saver){};

  /**
   * @brief Assign a name for this controller
   *
   * @param name A name reference. Names can be added or overwritten
   */
  virtual void GetMethodName(std::string &name){};

  /** ================================== */

  ModuleType type_; /* Constraint or Objective type */

protected:
  predictive_configuration *config_; /* Configuration parameters */
  VehicleRegion *vehicle_;           /* Collision region model */
};

#endif // __TYPES_H__