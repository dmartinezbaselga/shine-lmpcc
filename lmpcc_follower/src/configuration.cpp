#include <lmpcc_follower/configuration.h>

bool FollowerConfig::Initialize()
{
  ros::NodeHandle nh;

  retrieveParameter(nh, "follower/debug_output", debug_output_);

  if (debug_output_)
    ROS_INFO("Reading Parameters");

  retrieveParameter(nh, "follower/debug_solver", debug_solver_);

  retrieveParameter(nh, "follower/clock_frequency", clock_frequency_);

  retrieveParameter(nh, "follower/plan_reference_index", plan_reference_index_);
  retrieveParameter(nh, "follower/Kp_v", Kp_v_);
  retrieveParameter(nh, "follower/Kp_delta", Kp_delta_);
  retrieveParameter(nh, "follower/Kd_v", Kd_v_);
  retrieveParameter(nh, "follower/Kd_delta", Kd_delta_);
  retrieveParameter(nh, "follower/Kp_x", Kp_x_);
  retrieveParameter(nh, "follower/Kp_stanley", Kp_stanley_);

  return true;
}