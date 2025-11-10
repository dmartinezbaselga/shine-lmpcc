/**
 * @file dynamic_obstacle.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Class that holds obstacle information (position and prediction) and their collision regions.
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef DYNAMIC_OBSTACLE_H
#define DYNAMIC_OBSTACLE_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <lmpcc_msgs/gaussian.h>
#include <lmpcc_msgs/obstacle_gmm.h>

#include <ros_tools/helpers.h>

#include <lmpcc_tools/collision_region.h>

#include <Eigen/Dense>
#include <vector>
// class Disc; // Only used by reference

/**
 * @brief Class that holds obstacle information (position and prediction) and their collision regions.
 *
 * @param id_ id as defined by an external module (i.e., a simulation)
 * @param disc_start_id where do this obstacle's disc IDs start
 * @param discs_ the discs spanning the collision region of this obstacle
 * @param pose_ The position of the obstacle currently
 * @param prediction_ the predicted movement as Gaussian distribution
 * @todo Make discs easily iterable
 */
class DynamicObstacle
{

public:
    /**
     * @brief Create an obstacle with circular shape.
     *
     * @param id
     * @param disc_start_id start of obstacle discs as index (so that all discs can have a unique ID)
     * @param radius radius of the collision region
     */
    DynamicObstacle(int id, int disc_start_id, const Eigen::Vector2d &pos, double radius);

    // DynamicObstacle(int id, int disc_start_id, const geometry_msgs::Pose& pose, double radius);

    /**
     * @brief Create an obstacle with rectangular shape.
     *
     * @param id
     * @param disc_start_id start of obstacle discs as index (so that all discs can have a unique ID)
     * @param width
     * @param length
     * @param center_offset the state is offset with "center_offset" w.r.t. to the middle of the object
     * @param n_discs number of discs to model the collision region
     */
    DynamicObstacle(int id, int disc_start_id, const Eigen::Vector2d &pos, double width, double length, double center_offset, int n_discs);

    friend std::ostream &operator<<(std::ostream &out, const DynamicObstacle &obs)
    {
        if (obs.id_ == -1)
            return out << "Obstacle [DUMMY]:\t\t (" << obs.pose_.position.x << ", " << obs.pose_.position.y << ")";
        else
            return out << "Obstacle [" << obs.id_ << "]:\t\t (" << obs.pose_.position.x << ", " << obs.pose_.position.y << ")";
    }

public:
    int id_;

    std::vector<Disc> discs_; /** @todo: extend for more than one discs */
    std::vector<Disc> predicted_collision_regions_;

    geometry_msgs::Pose pose_;
    lmpcc_msgs::obstacle_gmm prediction_; // Deterministic or Gaussian

    /** @brief Get the expected collision region of this obstacle over the horizon (std::vec)
     * using the mean predictions of this obstacle (for the given mode or the only mode if 0) */
    std::vector<Disc> &GetPredictedCollisionRegions(int mode = 0);
    void ComputePredictedCollisionRegions(int mode = 0);

    /**
     * @brief Predict constant velocity for this obstacle if no prediction is given
     *
     * @param twist current twist of the obstacle
     * @param dt delta time of the prediction
     * @param N horizon of the prediction
     */
    void
    PredictConstantVelocity(const geometry_msgs::Twist &twist, double dt, int N, double one_sigma_radius = 1.0, double growth = 1.03);

    /**
     * @brief MoG Predictions for debugging purposes
     *
     * @param twist current twist of the obstacle
     * @param dt delta time of the prediction
     * @param N horizon of the prediction
     */
    void PredictDebugMixtureOfGaussian(const geometry_msgs::Twist &twist, double dt, int N);

    /**
     * @brief Dummy obstacle for padding of input data
     *
     * @param x x of the vehicle
     * @param y y of the vehicle
     * @param N horizon of the prediction
     */
    void DummyPrediction(const double x, const double y, int N);

    /**
     * @brief Load predictions into this obstacle
     *
     * @param prediction predictions to load
     */
    void LoadPredictions(const lmpcc_msgs::obstacle_gmm &prediction)
    {
        prediction_ = prediction; // Simple copy
    }

private:
    int previous_mode = -1;
};

#endif