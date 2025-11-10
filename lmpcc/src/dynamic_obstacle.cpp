#include "lmpcc/dynamic_obstacle.h"

DynamicObstacle::DynamicObstacle(int id, int disc_start_id, const Eigen::Vector2d &pos, double radius)
    : id_(id)
{
    discs_.emplace_back(disc_start_id, pos, 0., 0., radius); // No orientation and offset
}

DynamicObstacle::DynamicObstacle(int id, int disc_start_id, const Eigen::Vector2d &pos, double width, double length, double center_offset, int n_discs)
    : id_(id)
{
    ROSTOOLS_ASSERT(n_discs == 1, "Obstacles with more than one disc are not properly supported.");

    // Compute the offsets of the discs
    double offset = 0.;
    // std::vector<double> offsets;
    // for (int i = 0; i < 1; i++)
    // {
    // offsets.push_back(-center_offset - length / 2. + length / (n_discs) * (i + 0.5));
    // }

    // The radius is fitted to contain the full obstacle
    double radius = std::sqrt(std::pow(offset, 2.) + std::pow(width / 2., 2.));

    // Create discs for the obstacle
    // for (int i = 0; i < n_discs; i++)
    // {
    discs_.emplace_back(disc_start_id, pos, 0., offset, radius); // disc_start_id + i, offsets[i], radius);
    // }
}

void DynamicObstacle::ComputePredictedCollisionRegions(int mode)
{
    ROSTOOLS_ASSERT((int)prediction_.gaussians.size() >= mode, "Predictions need to be provided to compute predicted collision regions!");

    predicted_collision_regions_.clear();

    auto &mean_poses = prediction_.gaussians[mode].mean.poses;

    for (size_t k = 0; k < mean_poses.size(); k++)
    {
        geometry_msgs::Pose &cur_pose = mean_poses[k].pose;

        predicted_collision_regions_.emplace_back(
            0,
            Eigen::Vector2d(cur_pose.position.x, cur_pose.position.y),
            cur_pose.orientation.z,
            0.,
            discs_[0].radius);
    }
}

std::vector<Disc> &DynamicObstacle::GetPredictedCollisionRegions(int mode)
{
    if (previous_mode != mode)
        ComputePredictedCollisionRegions(mode);

    return predicted_collision_regions_;
}

void DynamicObstacle::PredictConstantVelocity(const geometry_msgs::Twist &twist, double dt, int N, double one_sigma_radius, double growth)
{
    prediction_.gaussians.clear();
    prediction_.probabilities.clear();

    lmpcc_msgs::gaussian gaussian;
    gaussian.mean.poses.resize(N);
    gaussian.major_semiaxis.resize(N);
    gaussian.minor_semiaxis.resize(N);

    for (int t = 0; t < N; t++)
    {
        gaussian.mean.poses[t].pose.position.x = pose_.position.x + twist.linear.x * dt * t;
        gaussian.mean.poses[t].pose.position.y = pose_.position.y + twist.linear.y * dt * t;

        gaussian.major_semiaxis[t] = one_sigma_radius * std::pow(growth, (double)t); // How should this increase over the horizon
        gaussian.minor_semiaxis[t] = one_sigma_radius * std::pow(growth, (double)t);
    }

    prediction_.gaussians.push_back(gaussian);
    prediction_.probabilities.push_back(1.0);
}

void DynamicObstacle::PredictDebugMixtureOfGaussian(const geometry_msgs::Twist &twist, double dt, int N)
{
    prediction_.gaussians.clear();
    prediction_.probabilities.clear();

    int modes = 3;            // Number of modes
    double range = 3.14 / 6.; // In radians

    for (int mode = 0; mode < modes; mode++)
    {
        lmpcc_msgs::gaussian gaussian;
        gaussian.mean.poses.resize(N);
        gaussian.major_semiaxis.resize(N);
        gaussian.minor_semiaxis.resize(N);

        double heading = -range + (2. * range / ((double)modes)) * ((double)mode);
        Eigen::Matrix2d rotation_matrix = RosTools::rotationMatrixFromHeading(heading);
        Eigen::Vector2d rotated_twist = rotation_matrix * Eigen::Vector2d(twist.linear.x, twist.linear.y);

        for (int t = 0; t < N; t++)
        {
            gaussian.mean.poses[t].pose.position.x = pose_.position.x + rotated_twist(0) * dt * t;
            gaussian.mean.poses[t].pose.position.y = pose_.position.y + rotated_twist(1) * dt * t;

            gaussian.major_semiaxis[t] = discs_[0].radius * 2. / 4. * std::pow(1.03, (double)t); // How should this increase over the horizon
            gaussian.minor_semiaxis[t] = discs_[0].radius * 2. / 4. * std::pow(1.03, (double)t);
        }

        prediction_.gaussians.push_back(gaussian);
        prediction_.probabilities.push_back(1.0 / (double)modes);
    }
}

void DynamicObstacle::DummyPrediction(const double x, const double y, int N)
{
    prediction_.gaussians.clear();
    prediction_.probabilities.clear();

    lmpcc_msgs::gaussian gaussian;
    gaussian.mean.poses.resize(N);
    gaussian.major_semiaxis.resize(N);
    gaussian.minor_semiaxis.resize(N);

    for (int t = 0; t < N; t++)
    {
        gaussian.mean.poses[t].pose.position.x = x + 100.;
        gaussian.mean.poses[t].pose.position.y = y + 100.;

        gaussian.major_semiaxis[t] = 0.01;
        gaussian.minor_semiaxis[t] = 0.01;
    }

    prediction_.gaussians.push_back(gaussian);
    prediction_.probabilities.push_back(1.0);
}