#include "scenario/json_trajectory_sampler.h"

void JsonTrajectorySampler::Init(predictive_configuration *config, int index)
{
  config_ = config;

  LMPCC_WARN("Initializing JSON Trajectory Sampler [" << index << "]");

  file_path_ = ros::package::getPath("lmpcc") + "/samples/real/json/json_data_" + std::to_string(index) + ".json"; // path to json samples

  ReadSamples();
}

void JsonTrajectorySampler::ReadSamples()
{
  using namespace nlohmann;
  json j;
  std::string jk;
  int window = 20;

  LMPCC_INFO("\tJSON File: " << file_path_);
  std::ifstream file_read(file_path_);
  // Start the file reading
  file_read >> j;
  std::vector<float> v2_x(window);
  std::vector<float> v2_y(window);

  // stores the observables, and the x, y trajectories of each sample in a separate vector
  // Oscar: Up until sample size for faster loading
  for (int s = 0; s < std::min((int)j.size(), config_->sample_size_); s++)
  // for (int s = 0; s < (int)j.size(); s++)
  {
    float observable_value = roundf(float(j[std::to_string(s)]["Observable"]) * 10) / 10;

    batch_o_.push_back(observable_value);
    for (int k = 0; k < window; k++)
    {
      std::string trajectory_numberx = "x" + std::to_string(k);
      std::string trajectory_numbery = "y" + std::to_string(k);
      v2_x[k] = (float(j[std::to_string(s)]["Trajectory_x"][trajectory_numberx]));
      v2_y[k] = (float(j[std::to_string(s)]["Trajectory_y"][trajectory_numbery]));
    }

    batch_x_.push_back(v2_x);
    batch_y_.push_back(v2_y);
  }
  LMPCC_INFO("\tSamples: " << batch_x_.size());

  if (batch_x_.size() < config_->sample_size_)
    LMPCC_ERROR("The sample size is larger than the samples in this partition!");
}

std::vector<std::vector<float>> &JsonTrajectorySampler::GetSampleBatchX()
{

  // Pointer to vector holding x values
  return batch_x_; // Only one batch
}
std::vector<std::vector<float>> &JsonTrajectorySampler::GetSampleBatchY()
{

  // Pointer to vector holding x values

  return batch_y_; // Only one batch
}
std::vector<float> &JsonTrajectorySampler::GetSampleBatchO()
{

  // Pointer to vector holding Observables

  return batch_o_; // Only one batch
}
