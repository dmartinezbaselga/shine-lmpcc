#include <lmpcc/types.h>

// #include <Eigen/Dense>
// #include <geometry_msgs/Pose.h>
#include <jsk_rviz_plugins/OverlayText.h>
// #include <unordered_map>
// #include <vector>

// #include <tkspline/spline.h>

// #include <dynamic_obstacle.h>
// #include <lmpcc/PredictiveControllerConfig.h>
// #include <lmpcc_configuration.h>

// #include "lmpcc_solver/collision_region.h"

// #include "lmpcc_solver/SolverInclude.h"
// #include <boost/bind.hpp>

Path::Path(int length)
{
    x_.reserve(length);
    y_.reserve(length);
    psi_.reserve(length);
}

void Path::AddPose(const geometry_msgs::Pose &pose)
{
    x_.push_back(pose.position.x);
    y_.push_back(pose.position.y);
    psi_.push_back(RosTools::quaternionToAngle(pose));
}

void Path::Clear()
{
    x_.clear();
    y_.clear();
    psi_.clear();
}

CubicSpline::CubicSpline() { knot_distances_.push_back(0.); }

// Add a spline
void CubicSpline::AddSpline(double a, double b, double c, double d, double L)
{
    a_.push_back(a);
    b_.push_back(b);
    c_.push_back(c);
    d_.push_back(d);

    knot_distances_.push_back(L);
}

// Add a spline
void CubicSpline::AddSpline(const Eigen::Vector4d &abcd, double L)
{
    a_.push_back(abcd(0));
    b_.push_back(abcd(1));
    c_.push_back(abcd(2));
    d_.push_back(abcd(3));

    knot_distances_.push_back(L);
}

// Evaluate the spline at distance s
double CubicSpline::operator()(double s) const
{
    int spline_idx = -1;
    double spline_start = 0.;
    for (size_t i = 0; i < knot_distances_.size() - 1; i++)
    {

        // Find the correct interval for s, i.e., where s_i <= s < s_i+1 (also use = at the end to include the final point)
        // Beyond the spline, use the last segment
        if ((s >= knot_distances_[i] && s <= knot_distances_[i + 1]) || (i == knot_distances_.size() - 2))
        {
            spline_idx = i;
            spline_start = knot_distances_[i];

            break;
        }
    }

    if (s > knot_distances_.back() || spline_idx == -1)
    {
        ROS_WARN_STREAM("Spline evaluated outside of range (s = " << std::to_string(s) << ", max length = " << std::to_string(knot_distances_.back())
                                                                  << ") - spline_idx = " << spline_idx);
    }

    double ss = (s - spline_start); // / (knot_distances_[spline_idx + 1] - spline_start); // Normalized in [0, 1]
    return a_[spline_idx] * std::pow(ss, 3.) + b_[spline_idx] * std::pow(ss, 2.) + c_[spline_idx] * ss + d_[spline_idx];
}

/** @brief Add interface for retrieving a, b, c, d */
void CubicSpline::GetParameters(int index, double &a, double &b, double &c, double &d)
{
    if (index >= (int)a_.size())
        ROS_WARN_STREAM("GetParameters: Accessed parameter beyond cubic spline definition (accessed: " << index << ", size: " << a_.size() << ")");

    a = a_[index];
    b = b_[index];
    c = c_[index];
    d = d_[index];
}

double CubicSpline::GetSplineStart(int index)
{
    // Prevent any index higher than the second to last
    index = std::min(index, (int)knot_distances_.size() - 2);

    return knot_distances_[index];
}

double CubicSpline::GetSplineEnd(int index)
{
    if (index >= (int)knot_distances_.size() - 1)
        ROS_WARN_STREAM(
            "GetDistance: Accessed parameter beyond cubic spline definition (accessed: " << index << ", size: " << knot_distances_.size() - 1 << ")");

    index = std::min(index, (int)knot_distances_.size() - 1);
    return knot_distances_[index + 1];
}

// MAJOR TYPES
RealTimeData::RealTimeData(int n_dynamic_obstacles, int n_static_obstacles)
{
    dynamic_obstacles_.reserve(n_dynamic_obstacles);
    halfspaces_.reserve(n_static_obstacles);

    velocity_ = 0.;
    reference_velocity_.push_back(0.); // Always have some reference available
}

RealTimeData::~RealTimeData(){};

void RealTimeData::Print()
{
    ROS_WARN("========== Real Time Data =========");
    ROS_INFO_STREAM("- DYNAMIC OBSTACLES -");
    for (auto &obs : dynamic_obstacles_)
        ROS_INFO_STREAM(obs);

    ROS_INFO_STREAM("- STATIC OBSTACLES -");
    for (int k = 0; k < 0; k++)
    {
        for (auto &halfspace : halfspaces_[k])
        {
            ROS_INFO_STREAM("Halfspace at stage " << k << " [" << halfspace.A_(0) << ", " << halfspace.A_(1) << "] b = " << halfspace.b_);
        }
    }

    ROS_INFO_STREAM("- PATH -");
    for (size_t i = 0; i < path_.Get().x_.size(); i++)
        ROS_INFO_STREAM("Path Waypoint: " << path_.Get().x_[i] << ", " << path_.Get().y_[i] << ", " << path_.Get().psi_[i]);

    ROS_WARN("===================================");
}

Monitor::Monitor(ros::NodeHandle &nh) { text_publisher_ = nh.advertise<jsk_rviz_plugins::OverlayText>("/lmpcc/interface_status", 1); };

/** @brief Mark that the callback with name "name" is expected to be called in each iteration. If only once is enough, set persistent to true */
void Monitor::MarkExpected(const std::string &&name, bool persistent)
{
    received_[name] = false; // Adds it to the map

    if (persistent)
        persistent_[name] = true;
}

void Monitor::MarkReceived(const std::string &&name) { received_[name] = true; }

// Print the status of all the signals and reset
void Monitor::PrintStatus()
{
    std::string message = "Signal Status:\n";
    for (auto &signal : received_)
    {
        if (signal.second) // Check the boolean to which this signal maps (i.e., the received flag)
        {
            message += "\t" + std::string(signal.first) + ": Received\n";
            if (persistent_.find(signal.first) == persistent_.end()) // If this variable is not persistent, reset its value to not received
                signal.second = false;
        }
        else
            message += "\t" + std::string(signal.first) + ": MISSING\n";
    }

    // Show a message on the screen
    jsk_rviz_plugins::OverlayText overlay_msg;
    overlay_msg.text = message;
    overlay_msg.action = 0;
    overlay_msg.width = 100;
    overlay_msg.height = 150;
    overlay_msg.left = 10;
    overlay_msg.top = 80;
    overlay_msg.bg_color.a = 0.2;
    overlay_msg.line_width = 2;
    overlay_msg.text_size = 12.0;
    overlay_msg.font = "DejaVu Sans Mono";
    overlay_msg.fg_color.r = 0.098;
    overlay_msg.fg_color.g = 0.94;
    overlay_msg.fg_color.b = 0.94;
    overlay_msg.fg_color.a = 1.0;

    text_publisher_.publish(overlay_msg);
}

ControllerModule::ControllerModule(ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle)
{
    config_ = config;
    vehicle_ = vehicle;
    type_ = ModuleType::UNDEFINED;
}
