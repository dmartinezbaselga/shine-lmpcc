#ifndef __PID_STANLEY_H__
#define __PID_STANLEY_H__

#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <generated_cpp/SolverInclude.h>

class PID_Stanley
{
public:
  PID_Stanley(ros::NodeHandle &nh);

  void CalculateControls(const double time_since_loading_trajectory); // double planner_prediction_integrator_stepsize); // For try out with speed differentiation

  void SetReference(const PriusDynamicsState &reference);

  void SetState(const PriusDynamicsState &state);

  void SetKp_X(const double K);
  void SetKp_Stanley(const double K);
  void SetKp_V(const double K);
  void SetKp_Delta(const double K);
  void SetKi_V(const double K);
  void SetKi_Delta(const double K);
  void SetKd_V(const double K);
  void SetKd_Delta(const double K);

  void ResetErrorIntegral(); // To reset the integral term of the PID controllers to 0

  PriusDynamicsState GetError();
  double GetControlSpeed();
  double GetControlSteeringAngle();
  double GetControlAcceleration();
  double GetControlSteeringRate();

private:
  ros::NodeHandle &nh_;
  ros::Publisher control_speed_pub_ = nh_.advertise<std_msgs::Float32>("pid_stanley/control_speed", 1000);
  ros::Publisher control_steering_angle_pub_ = nh_.advertise<std_msgs::Float32>("pid_stanley/control_steering_angle", 1000);
  ros::Publisher control_acceleration_pub_ = nh_.advertise<std_msgs::Float32>("pid_stanley/control_acceleration", 1000);
  ros::Publisher control_steering_rate_pub_ = nh_.advertise<std_msgs::Float32>("pid_stanley/control_steering_rate", 1000);

  PriusDynamicsState reference_, state_, error_, previous_error_;
  double sum_error_v_, sum_error_delta_;
  double Kp_x_, Kp_stanley_, Kp_v_, Kp_delta_;          // P-Gain
  double Ki_v_, Ki_delta_;                              // I-Gain
  double Kd_v_, Kd_delta_;                              // D-Gain
  double control_speed_, control_steering_angle_;       // Calculated control signal voor vx_delta control
  double control_acceleration_, control_steering_rate_; // Calculated control signal voor ax_omega control

  const double MAX_SPEED_ = 10.0;
  const double MIN_SPEED_ = 0.0;
  const double MAX_ACCELERATION_ = 1.0;
  const double MIN_ACCELERATION_ = -1.0;
  const double MAX_STEERING_ANGLE_ = 30 * 3.1415 / 180;  //  30 degrees
  const double MIN_STEERING_ANGLE_ = -30 * 3.1415 / 180; // -30 degrees
  const double MAX_STEERING_RATE_ = 0.2;
  const double MIN_STEERING_RATE_ = -0.2;
};

#endif // __PID_STANLEY_H__