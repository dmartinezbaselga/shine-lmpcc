#include "modules_constraints/gaussian_constraints.h"

#include <lmpcc/lmpcc_configuration.h>
#include <lmpcc_solver/SolverInclude.h>

GaussianConstraints::GaussianConstraints(ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle) : ControllerModule(nh, config, vehicle)
{
  type_ = ModuleType::CONSTRAINT;

  LMPCC_WARN_ALWAYS("Initializing Gaussian Constraints");
  LMPCC_WARN_ALWAYS("\tRisk: " << config_->risk_);

  // Initialise the visualisation
  ros_markers_.reset(new RosTools::ROSMarkerPublisher(nh, "lmpcc/gaussian_constraints/markers", config_->target_frame_, 200));
}

void GaussianConstraints::Update(SolverInterface *solver_interface, RealTimeData &data) { obstacles_ = (std::vector<DynamicObstacle> *)(&data.dynamic_obstacles_); }

void GaussianConstraints::SetParameters(SolverInterface *solver_interface, const RealTimeData &data, int k_solver, int &param_idx)
{

  assert(k_solver > 0);
  int k = k_solver - 1;
  // Compensate for plan displacement!

  if (k == 0)
  {
    LMPCC_INFO("Gaussian Obstacles: Inserting Constraints")
  }

  // ONLY APPLICABLE FOR THE CURRENT PEDESTRIAN SIMULATOR (to size the modes)
  double major = 0.;
  for (int i = 0; i <= k; i++)
  {
    major = std::sqrt(std::pow(major, 2.0) + std::pow(config_->manual_noise_ * solver_interface->DT, 2.));
  }

  // The position for each obstacle, each gaussian and each disc of the obstacle

  // double chi = RosTools::ExponentialQuantile(0.5, 1.0 - config_->risk_);
  solver_interface->setParameter(k_solver, param_idx, major);                                                      // sigma x
  solver_interface->setParameter(k_solver, param_idx, major);                                                      // sigma y
  solver_interface->setParameter(k_solver, param_idx, config_->risk_ /* / ((double)solver_interface->FORCES_N)*/); // epsilon
  // solver_interface->setParameter(k, param_idx + 3, config_->ego_w_ / 2.); // r_vehicle
  solver_interface->setParameter(k_solver, param_idx, config_->r_VRU_); // r_obstacle (assumed the same for all)

  // param_idx += 4;
  int id = 0;
  std::vector<DynamicObstacle> copied_obstacles = data.dynamic_obstacles_;

  for (auto &obstacle : copied_obstacles)
  {
    for (size_t gaussian_idx = 0; gaussian_idx < obstacle.prediction_.gaussians.size(); gaussian_idx++)
    {
      auto &collision_regions = obstacle.GetPredictedCollisionRegions(gaussian_idx);

      // Could give a warning here if there are not enough obstacles in the ellipsoidal solver

      if (collision_regions.size() >= 0)
      {
        auto &disc = collision_regions[k];
        // std::cout << "For risk of " << config_->risk_ << ", normalcdf = " << evaluate1DCDF(1.0 -
        // config_->risk_) << std::endl;
        solver_interface->setParameter(k_solver, param_idx, disc.x);
        solver_interface->setParameter(k_solver, param_idx, disc.y);
        // param_idx += 2;
        id++;
      }
      else
      {
        // Dummies
        if (k == 1)
          LMPCC_INFO("LMPCC: Adding dummies for gaussian constraints");

        solver_interface->setParameter(k_solver, param_idx, solver_interface->InitialPlan(k).x() + 100.);
        solver_interface->setParameter(k_solver, param_idx, solver_interface->InitialPlan(k).y());
        // param_idx += 2;
        id++;
      }
    }
  }

  if (config_->debug_boolean1_)
  {
    for (; id < 21 * config_->max_obstacles_ /*21*/; id++)
    {
      // Dummies
      if (k == 0)
        LMPCC_INFO("LMPCC: Adding dummies for gaussian constraints");

      solver_interface->setParameter(k_solver, param_idx, solver_interface->InitialPlan(k).x() + 100.);
      solver_interface->setParameter(k_solver, param_idx, solver_interface->InitialPlan(k).y());
      // param_idx += 2;
    }
  }
}

void GaussianConstraints::Visualize()
{
  if (config_->draw_ellipsoids_)
    VisualizeGaussians();

  ros_markers_->publish();
}

void GaussianConstraints::VisualizeGaussians()
{
  LMPCC_INFO("Visualizing gaussian obstacles");
  if (obstacles_->size() == 0 || obstacles_->at(0).prediction_.gaussians[0].mean.poses.size() == 0)
  {
    LMPCC_WARN("\tNo obstacles received. Skip visualization of ellipsoids.");
    return;
  }
  RosTools::ROSPointMarker &ellipsoid = ros_markers_->getNewPointMarker("CYLINDER");
  for (auto &obstacle : *obstacles_)
  {
    for (size_t gaussian_idx = 0; gaussian_idx < obstacle.prediction_.gaussians.size(); gaussian_idx++)
    {
      double major = 0.;

      for (uint k = 0; k < config_->indices_to_draw_.size(); k++)
      {
        major = std::sqrt(std::pow(major, 2.0) + std::pow(config_->manual_noise_ * 0.2, 2.));

        // double sigma = 3.0 / config_->received_object_sigma_;
        ellipsoid.setColorInt(k, (int)config_->indices_to_draw_.size(), 0.5);

        const int &index = config_->indices_to_draw_[k];
        auto &cur_disc = obstacle.GetPredictedCollisionRegions(gaussian_idx)[index];

        // Set the dimensions (constraint is + r on both axes) //
        double chi = RosTools::ExponentialQuantile(0.5, 1.0 - config_->risk_);

        ellipsoid.setScale(2 * (major * std::sqrt(chi) + cur_disc.radius), // std::sqrt(1. / mp),
                           2 * (major * std::sqrt(chi) + cur_disc.radius), 0.2);

        ellipsoid.addPointMarker(Eigen::Vector3d(cur_disc.x, cur_disc.y, 0.01));
      }
    }
  }
}
