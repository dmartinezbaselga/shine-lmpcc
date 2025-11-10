#include "scenario/sampler.h"

#include <random>

Sampler::Sampler() {}

void Sampler::Init(ros::NodeHandle &nh, predictive_configuration *config)
{
  config_ = config;

  LMPCC_WARN("Initializing Sampler...");

  S_ = config_->sample_size_;
  R_ = config_->removal_count_;
  B_ = config_->batch_count_;

  samples_ready_ = false;
  samples_requested_ = 0;

  // GaussianSampler::Get().Init(config); // This classed has been mostly replaced with GaussianSampler

  sample_size_pub_ = nh.advertise<std_msgs::Int32>("/lmpcc/sample_size", 1);

  // k, v, s
  trajectory_samples_.resize(predictive_configuration::N);
  for (size_t k = 0; k < predictive_configuration::N; k++)
  {
    trajectory_samples_[k].resize(config_->max_obstacles_);
    for (int v = 0; v < config_->max_obstacles_; v++)
    {
      trajectory_samples_[k][v].resize(2);
      trajectory_samples_[k][v][0] = Eigen::VectorXd::Ones(S_) * 100.0;
      trajectory_samples_[k][v][1] = Eigen::VectorXd::Ones(S_) * 100.5;
    }
  }

  if (config_->use_real_samples_)
  {
    std::string meta_path = ros::package::getPath("lmpcc") + "/samples/real/json/meta";
    std::ifstream meta_file(meta_path);

    int partition_size;
    meta_file >> partition_size;
    LMPCC_WARN_ALWAYS("Number of Partitions: " << partition_size);

    json_real_samples_.resize(partition_size); // categorizer_.GetSize());

#pragma omp parallel for
    for (size_t i = 0; i < json_real_samples_.size(); i++)
    {
      if (ros::ok())                            // To prevent slow stopping
        json_real_samples_[i].Init(config_, i); // Read real samples (velocity samples!)
    }
    largest_sample_size_ = config_->sample_size_;
  }
  // else if (config_->use_real_samples_)
  // {
  //     std::string meta_path = ros::package::getPath("lmpcc") + "/samples/real/meta";
  //     std::ifstream meta_file(meta_path);

  //     // Check if the file is okay
  //     if (!meta_file.good())
  //         throw std::runtime_error("Sampler: Meta file is missing!");

  //     // CODE FOR THE PARTITIONS! CURRENTLY DISABLED
  //     // Read the partition size

  //     else if (config_->use_real_samples_)
  //     {
  //         int partition_size;
  //         meta_file >> partition_size;

  //         ROS_WARN_STREAM("Number of Partitions: " << partition_size);

  //         real_samples_.resize(partition_size); // categorizer_.GetSize());
  //         for (size_t i = 0; i < real_samples_.size(); i++)
  //         {
  //             real_samples_[i].Init(config_, i); // Read real samples (velocity samples!)
  //         }
  //         largest_sample_size_ = real_samples_[0].GetSampleBatch()[0][0][0].size();
  //     }
  // }

  prev_poses_.resize(config_->max_obstacles_);
  velocities_.resize(config_->max_obstacles_);

  ROS_WARN("Sampler: Initialized");
}

/* Initialize the samples */
/* Output: trajectory_samples_ or stage_samples_ */
void Sampler::initScenarios()
{
  // Allocate space for the scenario data
  gaussian_samples_.resize(B_);

  // Read scenarios if they exist (pick some out of the database)
  bool scenario_file_exists = readScenariosFromFile();

  if (!scenario_file_exists || config_->build_database_)
  {

    LMPCC_INFO("Sampler: Generating a database with samples");

    // Generate new samples
    generateSampleDatabase();

    // And read them when ready
    readScenariosFromFile();
  }

  // Find the largest sample size
  largest_sample_size_ = -1;
  for (int b = 0; b < B_; b++)
  {
    if ((int)gaussian_samples_[b].size() > largest_sample_size_)
    {
      largest_sample_size_ = gaussian_samples_[b].size();
    }
  }
}

//
void Sampler::generateSampleDatabase()
{

  // Allocate space for the new samples and the extremest samples
  std::vector<std::vector<Eigen::Vector2d>> database;
  std::vector<std::vector<int>> extreme_sample_indices;
  database.resize(config_->sample_size_);
  extreme_sample_indices.resize(config_->sample_size_);
  for (int b = 0; b < config_->sample_size_; b++)
  {
    database[b].resize(S_);
    extreme_sample_indices[b].resize(2);

    // Sample Gaussian samples
    sampleGaussian(database[b]);

    // Prune only when using stage wise sampling
    if (!config_->use_trajectory_sampling_)
    {

      // Find the larges deviation in x,y
      findExtremeSamples(database[b], extreme_sample_indices[b]);

      // Prune scenarios! Save the left over sample count
      pruneScenarios(database[b], extreme_sample_indices[b]);
    }

    // Save scenarios
    saveScenarios(database[b], extreme_sample_indices[b], b);
  }
}

/* Sample new scenarios at random */
void Sampler::sampleGaussian(std::vector<Eigen::Vector2d> &samples_out)
{
  // Initialize variables
  double truncated_cap = 0.0;

  // If we sample a truncated Gaussian, compute the range modifier
  if (config_->truncated_)
  {
    truncated_cap = std::exp(-std::pow(config_->truncated_radius_, 2.0) / 2.0);
  }

  // Draw scenarios
  for (int s = 0; s < S_; s++)
  {

    // Generate uniform random numbers in 2D
    samples_out[s] = Eigen::Vector2d(rand_.Double(), rand_.Double());

    // In the case of truncated gaussian distribution, we modify the random sampled variables
    if (config_->truncated_)
    {
      samples_out[s](0) = samples_out[s](0) * (1.0 - truncated_cap) + truncated_cap;
    }

    // Convert them to a Gaussian
    RosTools::uniformToGaussian2D(samples_out[s]);
  }

  // Sort the samples on their distance to the mean of the distribution
  std::sort(samples_out.begin(), samples_out.end(), [](const Eigen::Vector2d &a, const Eigen::Vector2d &b) { return a.squaredNorm() > b.squaredNorm(); });
}

void Sampler::findExtremeSamples(const std::vector<Eigen::Vector2d> &samples, std::vector<int> &extreme_sample_indices)
{

  double max_x = 0.0;
  double max_y = 0.0;

  // Go through all samples
  for (int s = 0; s < S_; s++)
  {
    // Check if this is the furthest x point
    double abs_current_x = std::abs(samples[s](0));
    if (abs_current_x > max_x)
    {
      extreme_sample_indices[0] = s;
      max_x = abs_current_x;
    }

    // Check if this is the furthest x point
    double abs_current_y = std::abs(samples[s](1));
    if (abs_current_y > max_y)
    {
      extreme_sample_indices[1] = s;
      max_y = abs_current_y;
    }
  }
}

// Prune samples for efficiency
void Sampler::pruneScenarios(std::vector<Eigen::Vector2d> &samples, const std::vector<int> &extreme_sample_indices)
{
  // General idea: sample points in a circle around the distribution, check which ones are never close enough
  // Initialize vectors that we need
  std::vector<double> distances;
  distances.resize(S_);

  // Vector of indices that are close enough at some point
  std::vector<bool> used_indices(S_, false);

  std::vector<int> all_indices;
  all_indices.resize(S_);

  // Idea: Sample a few points around the circle, check if the scenario is used in that case
  for (int point_it = 0; point_it < 50; point_it++)
  {

    double angle = 2.0 * M_PI / 50.0 * (double)point_it; // Space the angle

    // Compute the associated point
    Eigen::Vector2d point(std::abs(samples[extreme_sample_indices[0]](0)) * std::cos(angle),
                          std::abs(samples[extreme_sample_indices[1]](1)) * std::sin(angle)); // rx * cos(theta) , ry * sin(theta)

    // For all samples, compute the distance
    for (int s = 0; s < S_; s++)
    {
      distances[s] = RosTools::dist(point, samples[s]);
      all_indices[s] = s;
    }

    // Sort to find the smallest distances to our point outside (keep track of indices!)
    std::sort(all_indices.begin(), all_indices.end(), [&distances](const int &a, const int &b) { return distances[a] < distances[b]; });

    // The closest l+R are marked as used
    for (int i = 0; i < config_->polygon_checked_constraints_ + config_->removal_count_; i++)
    {
      used_indices[all_indices[i]] = true;
    }
  }

  int batch_prune_count = 0;

  // Find the first index marked as used, starting from the end (remember sorted from large distance to small distance to mean)
  for (size_t s = S_ - 1; s >= 0; s--)
  {
    if (used_indices[s] == true)
    {
      batch_prune_count = s;
      break;
    }
  }

  // Resize the samples to those not pruned
  samples.resize(batch_prune_count);

  // std::cout << "total removed scenarios : " << S_ - batch_prune_count << "/" << S_ << " (" << (double)(S_ - batch_prune_count) / (double)S_ * 100.0 << ")" << std::endl;
}

bool Sampler::readScenariosFromFile()
{
  gaussian_samples_.resize(B_);
  extreme_sample_indices_.resize(B_);

  bool read_success = false;
  // For all batches that we need
  for (int b = 0; b < B_; b++)
  {
    // Allocate space for this batch
    gaussian_samples_[b].resize(S_);
    extreme_sample_indices_[b].resize(2);

    // Select a batch at random
    int batch_select = rand_.Int(config_->sample_size_);

    // Read scenarios from that batch
    read_success = readScenarios(gaussian_samples_[b], extreme_sample_indices_[b], batch_select);

    // If no samples exist, do not try for other batches
    if (!read_success)
      return false;
  }

  LMPCC_INFO("Sampler: Samples succesfully read from file");

  return true;
}

// Save a batch of scenarios
void Sampler::saveScenarios(const std::vector<Eigen::Vector2d> &samples, const std::vector<int> &extreme_sample_indices, int batch_index)
{

  // Get the package path
  std::string path = ros::package::getPath("lmpcc");

  path += "/samples/";

  if (boost::filesystem::create_directories(path))
    ROS_INFO_STREAM("Data Saver: Creating Directory Path: " << path);

  path += "scenario_batch_";

  if (config_->truncated_)
    path += "truncated_at_" + std::to_string((int)config_->truncated_radius_);

  path += "S" + std::to_string(config_->sample_size_) + "_R" + std::to_string(config_->removal_count_) + "_" + std::to_string(batch_index) + ".txt";

  // Setup a file stream
  std::ofstream scenario_file;

  ROS_INFO_STREAM("ScenarioVRU: Saving scenario batch " << batch_index << " to\n" << path);
  scenario_file.open(path);

  // Write scenarios to the file
  for (size_t s = 0; s < samples.size(); s++)
  {
    char str[39];

    // File will have two doubles with whitespace per line
    sprintf(str, "%.12f %.12f\n", samples[s](0), samples[s](1));
    scenario_file << str;
  }

  // Add the far index at the end
  char str[39];

  sprintf(str, "%d %d", extreme_sample_indices[0], extreme_sample_indices[1]);
  scenario_file << str;

  // Close the file
  scenario_file.close();
}

bool Sampler::readScenarios(std::vector<Eigen::Vector2d> &samples, std::vector<int> &extreme_sample_indices, int batch_index)
{

  // Get the package path
  std::string path = ros::package::getPath("lmpcc");

  // Save based on truncated or not truncated
  path += "/samples/";

  path += "scenario_batch_";

  if (config_->truncated_)
    path += "truncated_at_" + std::to_string((int)config_->truncated_radius_);

  path += "S" + std::to_string(config_->sample_size_) + "_R" + std::to_string(config_->removal_count_) + "_" + std::to_string(batch_index) + ".txt";

  // Start the file reading
  std::ifstream scenario_file(path);

  // Check if the file is okay
  if (!scenario_file.good())
  {
    LMPCC_INFO("Sampler:: No samples found!");

    return false;
    // throw std::runtime_error("Could not read scenario batch from database!");
  }

  // Initialize the reading variables
  double x, y;
  int s = 0;

  while (scenario_file >> x >> y)
  {
    samples[s] = Eigen::Vector2d(x, y);
    s++;
  }

  // Far index is at the end
  extreme_sample_indices[0] = (int)samples[s - 1](0);
  extreme_sample_indices[1] = (int)samples[s - 1](1);
  // s is index but is +1, size is +1, extreme_sample_indices is also -1
  samples.resize(s - 1);

  return true;
}

std::vector<trajectory_sample> *Sampler::SamplePredictions() { return &trajectory_samples_; }

void Sampler::LoadPredictions(const lmpcc_msgs::lmpcc_obstacle_array &obstacle_msgs)
{
  PROFILE_FUNCTION();

  // Ordered: ped (mix, mix, mix) | ped (mix, mix, mix) ....
  for (size_t v = 0; v < obstacle_msgs.lmpcc_obstacles.size() / ((int)3); v++)
  {
    for (int s = 0; s < S_; s++)
    {
      // each sample currently just picks one of the modes
      int pick = rand_.Int(3);

      nav_msgs::Path path = obstacle_msgs.lmpcc_obstacles[v * 3 + pick].trajectory;

      // For all stages
      for (uint k = 0; k < predictive_configuration::N; k++)
      {

        // int temp_fix = k;
        // temp_fix = std::min(temp_fix, 14);
        // Adapt the gaussian random number to this sigma and mu
        trajectory_samples_[k][v][0](s) = path.poses[k].pose.position.x; // + rand_.Double() * 0.1;
        trajectory_samples_[k][v][1](s) = path.poses[k].pose.position.y; // + rand_.Double() * 0.1; // Remove the rand later! ensures no parallel lines...
      }
    }
  }
}

// Currently for trajectory sampling only!
std::vector<trajectory_sample> *Sampler::SampleTrajectories(const lmpcc_msgs::lmpcc_obstacle_array &obstacle_msgs) { return &trajectory_samples_; }
// std::vector<SampleVector> *Sampler::SampleRealTrajectories(const lmpcc_msgs::lmpcc_obstacle_array &obstacle_msgs)

std::vector<SampleVector> *Sampler::SampleRealTrajectories(const lmpcc_msgs::lmpcc_obstacle_array &obstacle_msgs, const lmpcc_msgs::observation_partitioning &obs)
{
  PROFILE_FUNCTION();
  double dt = 0.2;
  // For all obstacles

  for (size_t v = 0; v < obstacle_msgs.lmpcc_obstacles.size(); v++)
  {
    // Find the partition that was assigned to this obstacle
    int ii = 0;
    for (size_t i = 0; i < obs.object_ids.size(); i++)
    {
      if (obs.object_ids[i] == obstacle_msgs.lmpcc_obstacles[v].id)
      {
        ii = int(obs.partitions[i]);
        break;
      }
    }
    if (ii == -1)
    {
      ROS_WARN_STREAM("Sample Partition not found! (ID = " << obstacle_msgs.lmpcc_obstacles[v].id << ")");
    }

    // If none was found, give it 0
    int index = ii;

    /**
     * @todo: Instead of config_->sample_size_, this should be the sample size of the current partition and the computation should be sized acoordingly
     *
     */
    // Retrieve the associated sample batch
    SampleVector *batch = real_samples_[index].GetSampleBatch(); // S samples
    // Initialize at current data point (x, y)
    trajectory_samples_[0][v][0] = Eigen::VectorXd::Ones((*batch)[0][0].size()) * obstacle_msgs.lmpcc_obstacles[v].trajectory.poses[0].pose.position.x;
    trajectory_samples_[0][v][1] = Eigen::VectorXd::Ones((*batch)[0][1].size()) * obstacle_msgs.lmpcc_obstacles[v].trajectory.poses[0].pose.position.y;

    for (size_t k = 0; k < predictive_configuration::N; k++)
    {

      // Take the 0th as previuos for the 0th index
      int prev_k = k == 0 ? k : k - 1;

      // Integrate velocities to positions
      trajectory_samples_[k][v][0] = trajectory_samples_[prev_k][v][0] + (*batch)[k][0] * dt;
      trajectory_samples_[k][v][1] = trajectory_samples_[prev_k][v][1] + (*batch)[k][1] * dt;
    }

    /**
     * @todo: Run trajectory_samples.resize(new_size) to make sure that the vector has the correct size (do it for x and y)
     * @note: Whatever the size of trajectory_samples_[k][v][0 and 1] is here is the size that will be taken into account
     */
  }

  // std::cout << "I GOT TO THIS LOOP real" << trajectory_samples_.size() << std::endl;

  return &trajectory_samples_;
}

// void Sampler::SampleJsonRealTrajectories(const lmpcc_msgs::lmpcc_obstacle_array &obstacle_msgs, const lmpcc_msgs::observation_partitioning &obs)
void Sampler::SampleJsonRealTrajectories(const RealTimeData &data)
{
  PROFILE_FUNCTION();

  bool online_partitioning = true;

  int N = predictive_configuration::N;
  double dt = predictive_configuration::SOLVER_DT;

  if (!online_partitioning)
  {
    // For each obstacle
    for (size_t v = 0; v < data.dynamic_obstacles_.size(); v++)
    {
      const auto &obstacle = data.dynamic_obstacles_[v];
      // double angle = 0.; // RosTools::quaternionToAngle(obstacle.pose_.orientation);

      // Find the partition through the map (or set to 0 if not found)
      const int partition_id = data.obstacle_partitions_.find(obstacle.id_) != data.obstacle_partitions_.end() ? data.obstacle_partitions_.at(obstacle.id_).id : 0;

      if (config_->debug_output_ && data.obstacle_partitions_.find(obstacle.id_) == data.obstacle_partitions_.end())
        ROS_WARN_STREAM("Sample Partition not found! (ID = " << obstacle.id_ << ")");

      // Retrieve the x y and observables for the partition as reference
      auto &batch_x = json_real_samples_[partition_id].GetSampleBatchX(); // [s, k]
      auto &batch_y = json_real_samples_[partition_id].GetSampleBatchY();
      // auto &batch_o = json_real_samples_[partition_id].GetSampleBatchO(); // Observables

      // OSCAR: Sample from the partitions at random
      // Sample layout: k, v, [x,y] (s)
      // Initialize the first trajectory sample
      trajectory_samples_[0][v][0] = Eigen::VectorXd::Ones(config_->sample_size_) * obstacle.pose_.position.x; // obstacle_msgs.lmpcc_obstacles[v].trajectory.poses[0].pose.position.x;
      trajectory_samples_[0][v][1] = Eigen::VectorXd::Ones(config_->sample_size_) * obstacle.pose_.position.y; // obstacle_msgs.lmpcc_obstacles[v].trajectory.poses[0].pose.position.y;

      for (int s = 0; s < config_->sample_size_; s++)
      {
        for (int k = 0; k < N; k++)
        {
          int prev_k = k == 0 ? 0 : k - 1; // At 0 we use the initialized value, otherwise we use the previous positions in the trajectory

          // Euler integrate the velocity to obtain a trajectory (taking the rotation of the pedestrian into account)
          trajectory_samples_[k][v][0](s) = trajectory_samples_[prev_k][v][0](s) + (double)batch_x[s][k] * dt;
          trajectory_samples_[k][v][1](s) = trajectory_samples_[prev_k][v][1](s) + (double)batch_y[s][k] * dt;
        }
      }
    }
  }
  else
  {
    // For each obstacle
    for (size_t v = 0; v < data.dynamic_obstacles_.size(); v++)
    {
      // THIS CODE IS COPIED CURRENTLY
      const auto &obstacle = data.dynamic_obstacles_[v];

      // Find the partition through the map (or set to 0 if not found)
      RealTimeData::Partition partition;
      if (data.obstacle_partitions_.find(obstacle.id_) != data.obstacle_partitions_.end())
        partition = data.obstacle_partitions_.at(obstacle.id_);
      else
        partition = {0, 0.};

      if (config_->debug_output_ && data.obstacle_partitions_.find(obstacle.id_) == data.obstacle_partitions_.end())
        ROS_WARN_STREAM("Sample Partition not found! (ID = " << obstacle.id_ << ")");

      // Retrieve the x y and observables for the partition as reference
      auto &batch_x = json_real_samples_[partition.id].GetSampleBatchX(); // [s, k]
      auto &batch_y = json_real_samples_[partition.id].GetSampleBatchY();
      auto &batch_o = json_real_samples_[partition.id].GetSampleBatchO(); // Observables

      // Retrieve the samples closest to the current observable within the selected partition
      online_partition_x_.clear();
      online_partition_y_.clear();
      online_partition_obs_.clear();
      sample_trajectories(batch_x, batch_y, batch_o, &online_partition_x_, &online_partition_y_, &online_partition_obs_,
                          config_->sample_size_ + config_->sample_size_ % 2, // If odd add one (this fixes the rounding later)
                          (float)partition.velocity);

      trajectory_samples_[0][v][0] = Eigen::VectorXd::Ones(config_->sample_size_) * obstacle.pose_.position.x;
      trajectory_samples_[0][v][1] = Eigen::VectorXd::Ones(config_->sample_size_) * obstacle.pose_.position.y;

      for (int k = 0; k < N; k++) // For all steps
      {
        int prev_k = k == 0 ? 0 : k - 1; // At 0 we use the initialized value, otherwise we use the previous positions in the trajectory

        for (int s = 0; s < config_->sample_size_; s++) // For all samples
        {
          // Euler integrate the velocity to obtain a trajectory (taking the rotation of the pedestrian into account)
          trajectory_samples_[k][v][0](s) = trajectory_samples_[prev_k][v][0](s) + (double)online_partition_x_[s][k] * dt;
          trajectory_samples_[k][v][1](s) = trajectory_samples_[prev_k][v][1](s) + (double)online_partition_y_[s][k] * dt;
        }
      }
    }
  }
  LMPCC_INFO_STREAM("Sampler: Real scenarios ready, format: [ k = " << trajectory_samples_.size() << " | v = " << trajectory_samples_[0].size() << " | s = " << trajectory_samples_[0][0][0].rows()
                                                                    << "]");
}

template <typename A, typename B, typename C>
void Sampler::sample_trajectories(std::vector<std::vector<A>> &a, std::vector<std::vector<B>> &b, std::vector<C> &c, std::vector<std::vector<A>> *ac, std::vector<std::vector<B>> *bc,
                                  std::vector<C> *cc, int samples, float Observable)
{

  if ((int)c.size() < samples)
  {
    copy(a.begin(), a.end(), back_inserter(*ac));

    copy(b.begin(), b.end(), back_inserter(*bc));
    copy(c.begin(), c.end(), back_inserter(*cc));
  }
  else
  {
    // How does this set the upper bound?
    auto low_bound = std::lower_bound(c.begin(), c.end(), Observable);
    auto index_to_start_x = low_bound - c.begin();
    auto index_to_end_x = c.end() - low_bound;

    if (index_to_end_x >= samples / 2 && index_to_start_x >= samples / 2)
    {
      copy(&a[index_to_start_x - samples / 2], &a[index_to_start_x + samples / 2], back_inserter(*ac)); // This is not safe, sample count may be odd
      copy(&b[index_to_start_x - samples / 2], &b[index_to_start_x + samples / 2], back_inserter(*bc));
      copy(&c[index_to_start_x - samples / 2], &c[index_to_start_x + samples / 2], back_inserter(*cc));
    }

    else if (index_to_end_x < samples / 2)
    {
      int samples_before = samples / 2 + (samples / 2 - index_to_end_x);
      int samples_after = samples / 2 - (samples / 2 - index_to_end_x);
      copy(&a[index_to_start_x - samples_before], &a[index_to_start_x + samples_after], back_inserter(*ac));
      copy(&b[index_to_start_x - samples_before], &b[index_to_start_x + samples_after], back_inserter(*bc));
      copy(&c[index_to_start_x - samples_before], &c[index_to_start_x + samples_after], back_inserter(*cc));
    }
    else if (index_to_start_x < samples / 2)
    {
      int samples_before = samples / 2 - (samples / 2 - index_to_start_x);

      int samples_after = samples / 2 + (samples / 2 - index_to_start_x);
      copy(&a[index_to_start_x - samples_before], &a[index_to_start_x + samples_after], back_inserter(*ac));
      copy(&b[index_to_start_x - samples_before], &b[index_to_start_x + samples_after], back_inserter(*bc));
      copy(&c[index_to_start_x - samples_before], &c[index_to_start_x + samples_after], back_inserter(*cc));
    }
  }
}
