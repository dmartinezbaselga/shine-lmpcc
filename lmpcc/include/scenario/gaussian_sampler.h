/**
 * @file gaussian_sampler.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Sampling of Gaussian distributions or Gaussian Mixture Models
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __GAUSSIAN_SAMPLER_H__
#define __GAUSSIAN_SAMPLER_H__

#include "lmpcc_msgs/gaussian.h"
#include "lmpcc_msgs/obstacle_array.h"

#include "std_msgs/Int32.h"

#include "lmpcc/dynamic_obstacle.h"
#include "lmpcc/lmpcc_configuration.h"
#include "ros_tools/data_saver.h"
#include "ros_tools/helpers.h"

#include "lmpcc/types.h"

/**
 * @brief Singleton class for sampling Gaussian distributions
 */
class GaussianSampler
{
public:
  GaussianSampler() { samples_ready_ = false; };
  // static GaussianSampler &Get()
  // {

  //     static GaussianSampler instance_;

  //     return instance_;
  // }

  // GaussianSampler(const GaussianSampler &) = delete;

public:
  bool standard_samples_ready_; // Internally: have the standard samples been prepared
  bool samples_ready_;          // Are the samples of the real distribution ready?

  /**
   * @brief Initialization. Reads or generates standard normal samples.
   *
   * @param config configuration parameters
   */
  void Init(predictive_configuration *config);

  /**
   * @brief Translate standard normal samples to the mean and variances of obstacle predictions
   *
   * @param obstacles_with_predictions Dynamic obstacles with populated predictions
   * @return std::vector<trajectory_sample>* Pointer to the scenarios
   */
  std::vector<trajectory_sample> *TranslateToMeanAndVariance(const std::vector<DynamicObstacle> &obstacles_with_predictions);

  /**
   * @brief Propagate samples over a horizon
   *
   * @param msg GMMs for all obstacles
   * @param dt integration step
   * @return std::vector<trajectory_sample>* Pointer to the scenarios
   */
  std::vector<trajectory_sample> *IntegrateAndTranslateToMeanAndVariance(const RealTimeData &data, const double dt);

  /**
   * @brief Load samples from an external message
   *
   * @param obstacle_msgs The message with samples (obstacle x sample x time x (x, y))
   * @param obstacles Obstacles for translating discs if necessary
   * @param sample_size_is_correct True if sample size is okay
   * @param sample_size_msg Message to send if sample size is not okay
   */
  void LoadSamples(const std_msgs::Float64MultiArray::ConstPtr &obstacle_msgs, const std::vector<DynamicObstacle> &obstacles, bool &sample_size_is_correct, std_msgs::Int32 &sample_size_ms);

  /**
   * @brief Get the samples
   *
   * @return std::vector<trajectory_sample>* pointer to the samples
   */
  std::vector<trajectory_sample> *GetSamples() { return &samples_; }; // By reference would be cleaner

  /**
   * @brief Returns the sample size of the first batch (do not use when sample size is varying)
   *
   * @return int sample size of the first batch
   */
  int SampleSize() const { return standard_samples_[0].size(); };

  // True if samples are prepared
  bool SamplesReady() const { return samples_ready_; };

private:
  //   GaussianSampler() { samples_ready_ = false; }; // Private constructor

  predictive_configuration *config_;
  RosTools::RandomGenerator random_generator_;

  // These are variables used in translating the samples online
  std::vector<std::vector<std::vector<Eigen::Matrix2d>>> R_, SVD_, Sigma_, A_;
  std::vector<std::vector<double>> sum_of_probabilities_;

  RosTools::DataSaver data_saver_; /* For saving and loading */

  std::vector<std::vector<Eigen::Vector2d>> standard_samples_; /* Batches of samples with mean 0 and variance of 1. */
  std::vector<trajectory_sample> samples_;                     /* Output samples, translated */

  /**
   * @brief Sample standard normal samples (i.e., mean 0, variance 1)
   */
  void SampleStandardNormal();

  /**
   * @brief Sample truncated standard normal samples (i.e., mean 0, variance 1)
   */
  void SampleTruncatedStandardNormal();

  /**
   * @brief Prune samples in the center of the distribution (only used in S-MPCC)
   */
  void Prune();

  /**
   * @brief Sort samples on distance from the center
   */
  void SortSamples();

  /**
   * @brief Resize samples based on pruned sample size
   */
  void ResizeSamples();

  /**
   * @brief Fit an ellipse that contains all samples
   *
   * @param samples the samples to fit on
   * @return Eigen::Vector2d axes of the ellipse
   */
  Eigen::Vector2d FitMaximumEllipse(const std::vector<Eigen::Vector2d> &samples);

  /**
   * @brief Save standard normal samples
   */
  void Save();

  /**
   * @brief Load standard normal samples
   *
   * @return true If file exists
   * @return false If no file exists
   */
  bool Load();

  /**
   * @brief Construct a file path for the samples
   *
   * @return std::string the file path
   */
  std::string GetFilePath();

  /**
   * @brief Construct a file name for the samples
   *
   * @return std::string the file name
   */
  std::string GetFileName();
};

#endif // __GAUSSIAN_SAMPLER_H__