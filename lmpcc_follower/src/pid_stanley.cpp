#include <lmpcc_follower/pid_stanley.h>

PID_Stanley::PID_Stanley(ros::NodeHandle &nh) : nh_(nh)
{
}

void PID_Stanley::CalculateControls(const double time_since_loading_trajectory)
{ // double planner_prediction_integrator_stepsize){

  double error_x_global = reference_.x() - state_.x();
  double error_y_global = reference_.y() - state_.y();
  error_.set_x(error_x_global * cos(state_.psi()) + error_y_global * sin(state_.psi()));
  error_.set_y(-error_x_global * sin(state_.psi()) + error_y_global * cos(state_.psi()));

  // Longitudinal PID
  double control_speed;
  double maximum_time_since_load_trajectory = 0.5; // In seconds
  if (time_since_loading_trajectory > maximum_time_since_load_trajectory)
  {
    control_speed = 0.;
    std::cout << "Minimal load trajectory time exceeded: (" << time_since_loading_trajectory << " > " << maximum_time_since_load_trajectory << ") emergency braking" << std::endl;
    ResetErrorIntegral();
  }
  else
  {
    control_speed = Kp_x_ * error_.x();
  }

  control_speed = std::max(MIN_SPEED_, std::min(MAX_SPEED_, control_speed));
  control_speed_ = control_speed;

  error_.set_v(control_speed - state_.v());
  sum_error_v_ += error_.v();
  double control_acceleration = Kp_v_ * error_.v() + Ki_v_ * sum_error_v_ + Kd_v_ * (error_.v() - previous_error_.v());

  control_acceleration = std::max(MIN_ACCELERATION_, std::min(MAX_ACCELERATION_, control_acceleration));
  control_acceleration_ = control_acceleration;

  // Steering Stanley and PID
  error_.set_psi(reference_.psi() - state_.psi());
  double control_steering_angle = error_.psi() + atan2(Kp_stanley_ * error_.y(), state_.v());

  control_steering_angle = std::max(MIN_STEERING_ANGLE_, std::min(MAX_STEERING_ANGLE_, control_steering_angle));
  control_steering_angle_ = control_steering_angle;

  error_.set_delta(control_steering_angle - state_.delta());
  sum_error_delta_ += error_.delta();

  double control_steering_rate = Kp_delta_ * error_.delta() + Ki_delta_ * sum_error_delta_ + Kd_delta_ * (error_.delta() - previous_error_.delta());
  control_steering_rate = std::max(MIN_STEERING_RATE_, std::min(MAX_STEERING_RATE_, control_steering_rate));
  control_steering_rate_ = control_steering_rate;

  previous_error_ = error_;

  std_msgs::Float32 control_speed_msg;
  control_speed_msg.data = control_speed_;
  control_speed_pub_.publish(control_speed_msg);

  std_msgs::Float32 control_steering_angle_msg;
  control_steering_angle_msg.data = control_steering_angle_;
  control_steering_angle_pub_.publish(control_steering_angle_msg);

  std_msgs::Float32 control_acceleration_msg;
  control_acceleration_msg.data = control_acceleration_;
  control_acceleration_pub_.publish(control_acceleration_msg);

  std_msgs::Float32 control_steering_rate_msg;
  control_steering_rate_msg.data = control_steering_rate_;
  control_steering_rate_pub_.publish(control_steering_rate_msg);
}

void PID_Stanley::SetReference(const PriusDynamicsState &reference)
{
  reference_ = reference;
}

void PID_Stanley::SetState(const PriusDynamicsState &state)
{
  state_ = state;
}

void PID_Stanley::SetKp_X(const double K)
{
  Kp_x_ = K;
}

void PID_Stanley::SetKp_Stanley(const double K)
{
  Kp_stanley_ = K;
}

void PID_Stanley::SetKp_V(const double K)
{
  Kp_v_ = K;
}

void PID_Stanley::SetKp_Delta(const double K)
{
  Kp_delta_ = K;
}

void PID_Stanley::SetKi_V(const double K)
{
  Ki_v_ = K;
}

void PID_Stanley::SetKi_Delta(const double K)
{
  Ki_delta_ = K;
}
void PID_Stanley::SetKd_V(const double K)
{
  Kd_v_ = K;
}

void PID_Stanley::SetKd_Delta(const double K)
{
  Kd_delta_ = K;
}

void PID_Stanley::ResetErrorIntegral()
{
  sum_error_v_ = 0.;
  sum_error_delta_ = 0.;
}

PriusDynamicsState PID_Stanley::GetError()
{
  return error_;
}

double PID_Stanley::GetControlSpeed()
{
  return control_speed_;
}

double PID_Stanley::GetControlSteeringAngle()
{
  return control_steering_angle_;
}

double PID_Stanley::GetControlAcceleration()
{
  return control_acceleration_;
}

double PID_Stanley::GetControlSteeringRate()
{
  return control_steering_rate_;
}