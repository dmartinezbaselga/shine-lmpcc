#include "scenario/trajectory_sampler.h"

void TrajectorySampler::Init(predictive_configuration *config, int index)
{
    ROS_WARN_STREAM("Initializing Trajectory Sampler [" << index << "]");
    config_ = config;
    file_path_ = ros::package::getPath("lmpcc") + "/samples/real/data_" + std::to_string(index);

    ReadSamples();
}

void TrajectorySampler::ReadSamples()
{
    std::cout << file_path_ << std::endl;
    // Start the file reading
    std::ifstream scenario_file(file_path_);

    // Check if the file is okay
    if (!scenario_file.good())
    {
        // return false;
        throw std::runtime_error("Could not read real scenario batch from database!");
    }

    // Read the size
    scenario_file >> data_size_;
    std::cout << "data size: " << data_size_ << std::endl;

    // Initialize variables
    all_samples_.resize(predictive_configuration::N);
    for (size_t k = 0; k < predictive_configuration::N; k++)
    {
        // x, y vectors
        all_samples_[k].resize(2);
        all_samples_[k][0] = Eigen::VectorXd::Zero(data_size_);
        all_samples_[k][1] = Eigen::VectorXd::Zero(data_size_);
    }

    // Read all samples from the file
    double x, y;
    int s = 0;
    int k = 0;
    while (scenario_file >> x >> y)
    {
        // std::cout << "at k = " << k << ", s = " << s << ": (" << x << ", " << y << ")" << std::endl;
        all_samples_[k][0](s) = x;
        all_samples_[k][1](s) = y;

        k += 1;
        if (k == (int)predictive_configuration::N)
        {
            s++;
            k = 0;
        }
    }

    // Now we construct subdata sets of the sample size which we can use directly online

    std::vector<int> random_indices;
    random_indices.resize(data_size_);
    std::iota(random_indices.begin(), random_indices.end(), 0); // List of indices up to data size

    batches_.resize(config_->batch_count_);
    for (size_t b = 0; b < batches_.size(); b++)
    {
        batches_[b].resize(predictive_configuration::N);

        // Shuffle the indices
        std::random_shuffle(random_indices.begin(), random_indices.end());

        // Save the samples
        for (size_t k = 0; k < predictive_configuration::N; k++)
        {
            batches_[b][k].resize(2);
            batches_[b][k][0] = Eigen::VectorXd(config_->sample_size_);
            batches_[b][k][1] = Eigen::VectorXd(config_->sample_size_);

            // Assign the first S to this batch
            for (int s = 0; s < config_->sample_size_; s++)
            {
                batches_[b][k][0](s) = all_samples_[k][0](random_indices[s]);
                batches_[b][k][1](s) = all_samples_[k][1](random_indices[s]);
            }
        }
    }
}

SampleVector *TrajectorySampler::GetSampleBatch()
{

    // Return a pointer to one of the batches
    // int batch_index = rand_.Int(batches_.size());

    return &batches_[0]; // Only one batch
}
