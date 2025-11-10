#ifndef __FOLLOWER_CONFIGURATION_H__
#define __FOLLOWER_CONFIGURATION_H__

#include <ros/ros.h>

/** Logging Pragmas */
#define FOLLOWER_INFO(msg)                      \
    if (config_->debug_output_)                 \
    {                                           \
        ROS_INFO_STREAM("[FOLLOWER]: " << msg); \
    }

#define FOLLOWER_WARN(msg)                      \
    if (config_->debug_output_)                 \
    {                                           \
        ROS_WARN_STREAM("[FOLLOWER]: " << msg); \
    }

#define FOLLOWER_ERROR(msg) ROS_ERROR_STREAM("[FOLLOWER]: " << msg)

#define FOLLOWER_INFO_STREAM(msg)               \
    if (config_->debug_output_)                 \
    {                                           \
        ROS_INFO_STREAM("[FOLLOWER]: " << msg); \
    }

#define FOLLOWER_WARN_STREAM(msg)               \
    if (config_->debug_output_)                 \
    {                                           \
        ROS_WARN_STREAM("[FOLLOWER]: " << msg); \
    }

#define FOLLOWER_SUCCESS(msg)                                        \
    if (config_->debug_output_)                                      \
    {                                                                \
        ROS_INFO_STREAM("\033[32m[FOLLOWER]: " << msg << "\033[0m"); \
    }

#define FOLLOWER_ERROR_STREAM(msg) ROS_ERROR_STREAM("[FOLLOWER]: " << msg)

#define FOLLOWER_INFO_ALWAYS(msg) ROS_INFO_STREAM("[FOLLOWER]: " << msg)
#define FOLLOWER_WARN_ALWAYS(msg) ROS_WARN_STREAM("[FOLLOWER]: " << msg)
#define FOLLOWER_SUCCESS_ALWAYS(msg) ROS_INFO_STREAM("\033[32m[FOLLOWER]: " << msg << "\033[0m");
#define FOLLOWER_HOOK std::cout << "FOLLOWER HOOK: " << __FILE__ << " Line " << __LINE__ << std::endl;

class FollowerConfig
{
public:
    FollowerConfig(){};

    /**
     * @brief intialize: Read all parameters from the ros parameter server
     * @return TRUE iff all parameter initialize successfully
     */
    bool Initialize();

    /************ CONFIGURATION VARIABLES **************/
    // DEBUG
    bool debug_output_;
    bool debug_solver_;
    double clock_frequency_; // hz clock Frequency for follower system

    // static double SOLVER_DT;
    // static unsigned int N;

    // double solver_timeout_;

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

private:
    /**
     * @brief Retrieve a parameter from the ROS parameter server, return false if it failed
     *
     * @tparam T Variable type
     * @param nh nodehandle
     * @param name Name of the parameter on the server
     * @param value Variable to store the read value in
     * @return true If variable exists
     * @return false If variable does not exist
     */
    template <class T>
    bool retrieveParameter(const ros::NodeHandle &nh, const std::string &name, T &value)
    {

        if (!nh.getParam(name, value))
        {
            ROS_ERROR_STREAM(" Parameter " << name << " not set on node " << ros::this_node::getName().c_str());
            throw std::runtime_error("Follower: Required Parameter Missing");
        }
        else
        {
            return true;
        }
    }

    /**
     * @brief Retrieve a parameter from the ROS parameter server, otherwise use the default value
     *
     * @tparam T Variable type
     * @param nh nodehandle
     * @param name Name of the parameter on the server
     * @param value Variable to store the read value in
     * @param default_value Default value to use if the variable does not exist
     */
    template <class T>
    void retrieveParameter(const ros::NodeHandle &nh, const std::string &name, T &value, const T &default_value)
    {

        if (!nh.getParam(name, value))
        {
            ROS_WARN_STREAM("Parameter \"" << name << "\" not set on " << ros::this_node::getName().c_str() << " -> using default value: " << default_value);
            value = default_value;
        }
    }

    template <class L>
    void retrieveParameter(const ros::NodeHandle &nh, const std::string &name, std::vector<L> &value, const std::vector<L> &default_value)
    {

        if (!nh.getParam(name, value))
        {
            ROS_WARN_STREAM("Parameter \"" << name << "\" not set on " << ros::this_node::getName().c_str() << " -> using default value with size: " << default_value.size());
            value = default_value;
        }
    }
};

#endif // __FOLLOWER_CONFIGURATION_H__