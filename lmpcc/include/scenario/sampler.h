/**
 * @file sampler.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Class has been replaced largely with GaussianSampler
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef SAMPLER_H
#define SAMPLER_H

#include <Eigen/Eigen>
#include "lmpcc/lmpcc_configuration.h"
#include "ros_tools/helpers.h"
#include <ros/ros.h>
#include "scenario/trajectory_sampler.h"
#include "scenario/json_trajectory_sampler.h"

#include "lmpcc_msgs/lmpcc_obstacle.h"
#include "lmpcc_msgs/lmpcc_obstacle_array.h"
#include "lmpcc_msgs/obstacle_array.h"
#include "lmpcc_msgs/obstacle_gmm.h"
#include "lmpcc_msgs/gaussian.h"
#include "lmpcc_msgs/observation_partitioning.h"
#include "lmpcc_solver/collision_region.h"

#include "ros_tools/helpers.h"

#include "lmpcc/lmpcc_configuration.h"
#include "lmpcc/dynamic_obstacle.h"
#include "scenario/gaussian_sampler.h"
#include "scenario/trajectory_sampler.h"

// typedef std::vector<Eigen::MatrixXd> trajectory_sample; // location per obstacle and time step
typedef std::vector<Eigen::Vector2d> stage_sample; // location per obstacle

/* Singleton Class for generating samples, saving and loading. */
/* Supports both sampling per stages and sampling of trajectories */
class Sampler
{

public:
    // Singleton function
    static Sampler &Get()
    {

        static Sampler instance_;

        return instance_;
    }

    Sampler(const Sampler &) = delete;

public:
    void Init(ros::NodeHandle &nh, predictive_configuration *config);
    bool SamplesReady() const { return samples_ready_; };

    /**
     * @brief Integrates uncertainty per stage to trajectory samples (assuming no covariance between stages)
     *
     * @param obstacle_msgs
     */
    // void IntegratePredictionsToTrajectories(const lmpcc_msgs::obstacle_array &obstacle_msgs, const double dt);

    // Necessary for now!
    void ResetSampleRequestCounter() { samples_requested_ = 0; };

    // std::vector<Eigen::Vector2d> &BatchReference(int batch_index);
    // std::vector<int> &ExtremeSampleIndices(int batch_index);

    // int LargestSampleSize() const;

private:
    // Private constructor!
    Sampler();

    bool samples_ready_;
    int samples_requested_;
    double largest_sample_size_;

    std::vector<std::vector<int>> extreme_sample_indices_; // Most extreme samples

    // S samples
    std::vector<trajectory_sample> trajectory_samples_; // Actual database

    // B_ Batches with S_ gaussian samples
    std::vector<std::vector<Eigen::Vector2d>> gaussian_samples_;

    // Publisher for communication with external sampler
    ros::Publisher sample_size_pub_;

    // Configuration
    predictive_configuration *config_;

    RosTools::RandomGenerator rand_;

    int S_; // sampling count
    int B_; // Batches
    int R_; // Scenario removal (maximum now)

    // Stuff for real samples
    std::vector<TrajectorySampler> real_samples_;
    std::vector<JsonTrajectorySampler> json_real_samples_;

    std::vector<Eigen::Vector2d> prev_poses_; // Move to interface
    std::vector<Eigen::Vector2d> velocities_;

    // General initialization function
    void initScenarios();

    // Sample generation and pruning
    void generateSampleDatabase();
    void sampleGaussian(std::vector<Eigen::Vector2d> &samples_out);
    void findExtremeSamples(const std::vector<Eigen::Vector2d> &samples, std::vector<int> &extreme_sample_indices);
    void pruneScenarios(std::vector<Eigen::Vector2d> &samples, const std::vector<int> &far_index);

    // Saving and loading
    bool readScenariosFromFile();
    void saveScenarios(const std::vector<Eigen::Vector2d> &samples, const std::vector<int> &far_index, int batch_index);
    bool readScenarios(std::vector<Eigen::Vector2d> &samples, std::vector<int> &far_index, int batch_index);

    std::vector<std::vector<float>> online_partition_x_, online_partition_y_; // This is bad for memory
    std::vector<float> online_partition_obs_;
    template <typename A, typename B, typename C>
    void sample_trajectories(std::vector<std::vector<A>> &a,
                             std::vector<std::vector<B>> &b,
                             std::vector<C> &c,
                             std::vector<std::vector<A>> *ac,
                             std::vector<std::vector<B>> *bc,
                             std::vector<C> *cc, int samples, float Observable);

public:
    std::vector<trajectory_sample> *SampleTrajectories(const lmpcc_msgs::lmpcc_obstacle_array &msg);

    std::vector<SampleVector> *SampleRealTrajectories(const lmpcc_msgs::lmpcc_obstacle_array &msg, const lmpcc_msgs::observation_partitioning &obs);
    // void SampleJsonRealTrajectories(const lmpcc_msgs::lmpcc_obstacle_array &msg, const lmpcc_msgs::observation_partitioning &obs);
    void SampleJsonRealTrajectories(const RealTimeData &data);

    void LoadPredictions(const lmpcc_msgs::lmpcc_obstacle_array &msg);
    void LoadSamples(const std_msgs::Float64MultiArray::ConstPtr &msg, const std::vector<DynamicObstacle> &obstacles);
    std::vector<trajectory_sample> *SamplePredictions();

    /*
        std::vector<trajectory_sample> *SampleTrajectories(const lmpcc_msgs::lmpcc_obstacle_array &msg);
        std::vector<trajectory_sample> *TranslateGaussianSamples(const lmpcc_msgs::lmpcc_obstacle_array &obstacles_msg);

        std::vector<SampleVector> *SampleRealTrajectories(const lmpcc_msgs::lmpcc_obstacle_array &msg, const lmpcc_msgs::observation_partitioning &obs);
        // std::vector<SampleVector> *SampleRealTrajectories(const lmpcc_msgs::lmpcc_obstacle_array &msg);
        void SampleJsonRealTrajectories(const lmpcc_msgs::lmpcc_obstacle_array &msg, const lmpcc_msgs::observation_partitioning &obs);

        void LoadPredictions(const lmpcc_msgs::lmpcc_obstacle_array &msg);
        void LoadSamples(const std_msgs::Float64MultiArray &msg);
        std::vector<trajectory_sample> *SamplePredictions();

        std::vector<Eigen::Vector2d> &BatchReference(int batch_index);
        std::vector<int> &ExtremeSampleIndices(int batch_index);
        std::vector<int> &JsonExtremeSampleIndices(int stage);

        int LargestSampleSize() const;*/
};

#endif