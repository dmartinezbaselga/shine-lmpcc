#ifndef JSON_TRAJECTORY_SAMPLER_H
#define JSON_TRAJECTORY_SAMPLER_H

#include <Eigen/Eigen>
#include "ros_tools/helpers.h"
#include "lmpcc/lmpcc_configuration.h"
#include <iostream>
#include <fstream>
#include "json.hpp"
#include <vector>
#include <bits/stdc++.h>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <cmath>
//  std::vector<std::vector<std::vector<float>>  batches_(2);
typedef std::vector<std::vector<Eigen::VectorXd>> SampleVector; // k | x/y | (s) location per obstacle and time step -X- -> No interaction: position along the trajectory, x/y
class JsonTrajectorySampler{


public:
    JsonTrajectorySampler(){};
    
        

private:
    predictive_configuration* config_;

    int data_size_;
    std::string file_path_;
    RosTools::RandomGenerator rand_;

    SampleVector batches_; // Subset of the database with the right size (batch, sample) (needs to copy because of the vectorxd class)

    int small;
    std::vector<float> batch_o_;
    std::vector<std::vector<float>> batch_x_;
    std::vector<std::vector<float>>  batch_y_;
     void ReadSamples();

public:
    void Init(predictive_configuration *config, int index);

    std::vector<std::vector<float>>& GetSampleBatchX();
    std::vector<std::vector<float>>& GetSampleBatchY();
    std::vector<float>& GetSampleBatchO();
    
     
};

#endif