/**
 * @file trajectory_sampler.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Deprecated class for sample reading
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef TRAJECTORY_SAMPLER_H
#define TRAJECTORY_SAMPLER_H

#include <Eigen/Eigen>
#include "ros_tools/helpers.h"
#include "lmpcc/lmpcc_configuration.h"

typedef std::vector<std::vector<Eigen::VectorXd>> SampleVector; // k | x/y | (s) location per obstacle and time step -X- -> No interaction: position along the trajectory, x/y
class TrajectorySampler
{

public:
    TrajectorySampler(){};

private:
    predictive_configuration *config_;

    int data_size_;
    std::string file_path_;
    RosTools::RandomGenerator rand_;

    SampleVector all_samples_;                                  // Database
    std::vector<SampleVector> batches_;                         // Subset of the database with the right size (batch, sample) (needs to copy because of the vectorxd class)
    std::vector<std::vector<SampleVector> *> assigned_batches_; // Currently assigned batches for all obstacles (vector of pointers to a batch)

    void ReadSamples();

public:
    void Init(predictive_configuration *config, int index);

    SampleVector *GetSampleBatch();
};

#endif