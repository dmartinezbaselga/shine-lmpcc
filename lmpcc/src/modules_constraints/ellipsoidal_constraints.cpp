#include "modules_constraints/ellipsoidal_constraints.h"

#include <lmpcc/lmpcc_configuration.h>
#include <lmpcc_solver/SolverInclude.h>

EllipsoidalConstraints::EllipsoidalConstraints(ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle)
    : ControllerModule(nh, config, vehicle)
{
    LMPCC_INFO("Initializing Ellipsoidal Constraints Module");
    type_ = ModuleType::CONSTRAINT;

    // Initialise the visualisation
    ros_markers_.reset(new RosTools::ROSMarkerPublisher(nh, "lmpcc/ellipsoid_constraints/markers", config_->target_frame_, 200));
}

void EllipsoidalConstraints::Update(SolverInterface *solver_interface, RealTimeData &data)
{
    obstacles_ = (std::vector<DynamicObstacle> *)(&data.dynamic_obstacles_);
}

void EllipsoidalConstraints::SetParameters(SolverInterface *solver_interface, const RealTimeData &data, int k_solver, int &param_idx)
{
    assert(k_solver > 0);
    // Compensate for plan displacement!
    int k = k_solver - 1; // Because the initial state is skipped, we need to do "-1"    bool deterministic = true;
    // Compensate for plan displacement!
    bool deterministic = true;

    if (k == 1)
    {
        LMPCC_INFO("Ellipsoid Obstacles: Setting Solver Parameters")
    }

    // ONLY APPLICABLE FOR THE CURRENT PEDESTRIAN SIMULATOR (to size the modes)

    double major = 0.;

    if (!deterministic)
    {
        for (int i = 0; i <= k; i++)
        {
            major = std::sqrt(std::pow(major, 2.0) + std::pow(0.3 * 0.2, 2.));
        }
    }

    // The position for each obstacle, each gaussian and each disc of the obstacle
    int id = 0;
    int k_obs;
    if (config_->synchronized_actuation_)
        k_obs = k == (int)solver_interface->FORCES_N - 1 ? (int)solver_interface->FORCES_N - 1 : k + 1; // one step forward
    else
        k_obs = k;

    double chi = RosTools::ExponentialQuantile(0.5, 1.0 - config_->risk_);

    // For thread safety!
    std::vector<DynamicObstacle> copied_obstacles = data.dynamic_obstacles_;

    for (auto &obstacle : copied_obstacles)
    {
        for (size_t gaussian_idx = 0; gaussian_idx < obstacle.prediction_.gaussians.size(); gaussian_idx++)
        {
            auto &collision_regions = obstacle.GetPredictedCollisionRegions(gaussian_idx);

            if (collision_regions.size() >= 0)
            {
                auto &disc = collision_regions[k_obs];
                // std::cout << "For risk of " << config_->risk_ << ", normalcdf = " << evaluate1DCDF(1.0 - config_->risk_) << std::endl;
                const geometry_msgs::Pose &pose = obstacle.prediction_.gaussians[gaussian_idx].mean.poses[k_obs].pose;
                solver_interface->setParameter(k_solver, param_idx, disc.x);
                solver_interface->setParameter(k_solver, param_idx, disc.y);
                solver_interface->setParameter(k_solver, param_idx, pose.orientation.z);

                // axis are assumed to be squared for now
                if (deterministic)
                {
                    solver_interface->setParameter(k_solver, param_idx, 0.);          // SQRT FOR CARLA
                    solver_interface->setParameter(k_solver, param_idx, 0.);          // Probably a *3 problem?
                    solver_interface->setParameter(k_solver, param_idx, 1.0);         // Extra parameter for the risk level (or set to "1" for not used)
                    solver_interface->setParameter(k_solver, param_idx, disc.radius); // Probably a *3 problem?
                }
                else
                {
                    solver_interface->setParameter(k_solver, param_idx, major);       // SQRT FOR CARLA
                    solver_interface->setParameter(k_solver, param_idx, major);       // Probably a *3 problem?
                    solver_interface->setParameter(k_solver, param_idx, chi);         // Extra parameter for the risk level (or set to "1" for not used)
                    solver_interface->setParameter(k_solver, param_idx, disc.radius); // Probably a *3 problem?
                }

                id++;
            }
            else
            {
                // Dummies
                if (k == 1)
                {
                    LMPCC_INFO("LMPCC: Adding dummies for ellipsoidal constraints");
                }

                solver_interface->setParameter(k_solver, param_idx, solver_interface->InitialPlan(k).x() + 100.);
                solver_interface->setParameter(k_solver, param_idx, solver_interface->InitialPlan(k).y());
                solver_interface->setParameter(k_solver, param_idx, 0.);
                solver_interface->setParameter(k_solver, param_idx, 0.1);
                solver_interface->setParameter(k_solver, param_idx, 0.1);
                solver_interface->setParameter(k_solver, param_idx, 1);   // Extra parameter for the risk level (or set to "1" for not used)
                solver_interface->setParameter(k_solver, param_idx, 0.0); // Extra parameter for the risk level (or set to "1" for not used)

                id++;
            }
        }
    }

    for (; id < config_->max_obstacles_ /** 21*/; id++)
    {
        // Dummies
        if (k == 1)
        {
            LMPCC_INFO("LMPCC: Adding dummies for ellipsoidal constraints");
        }

        solver_interface->setParameter(k_solver, param_idx, solver_interface->InitialPlan(k).x() + 100.);
        solver_interface->setParameter(k_solver, param_idx, solver_interface->InitialPlan(k).y());
        solver_interface->setParameter(k_solver, param_idx, 0.);
        solver_interface->setParameter(k_solver, param_idx, 0.1);
        solver_interface->setParameter(k_solver, param_idx, 0.1);
        solver_interface->setParameter(k_solver, param_idx, 1);
        solver_interface->setParameter(k_solver, param_idx, 0.0);
    }
}

//-------------------------------- VISUALIZATION -----------------------------------//
void EllipsoidalConstraints::Visualize()
{
    if (config_->draw_ellipsoids_)
        VisualizeEllipsoids();

    ros_markers_->publish();
}

// Visualise the predictions
void EllipsoidalConstraints::VisualizeEllipsoids()
{
    LMPCC_INFO("Visualizing ellipsoid obstacles");
    bool deterministic = true;

    // if (obstacles_->size() == 0 || obstacles_->at(0).prediction_.gaussians.size() == 0)
    // {
    //     LMPCC_WARN("\tNo obstacles received. Skip visualization of ellipsoids.");
    //     return;
    // }
    RosTools::ROSPointMarker &ellipsoid = ros_markers_->getNewPointMarker("CYLINDER");
    for (auto &obstacle : *obstacles_)
    {
        for (size_t mode_idx = 0; mode_idx < obstacle.prediction_.gaussians.size(); mode_idx++)
        {
            auto &collision_regions = obstacle.GetPredictedCollisionRegions(mode_idx);
            double major = 0.;

            for (uint k = 0; k < config_->indices_to_draw_.size(); k++)
            {
                major = deterministic ? 0. : std::sqrt(std::pow(major, 2.0) + std::pow(0.3 * 0.2, 2.));

                // double sigma = 3.0 / config_->received_object_sigma_;
                ellipsoid.setColorInt(k, (int)config_->indices_to_draw_.size(), 0.5);

                const int &index = config_->indices_to_draw_[k];

                // Set the dimensions (constraint is + r on both axes) //
                double chi = deterministic ? 1.0 : RosTools::ExponentialQuantile(0.5, 1.0 - config_->risk_); // Or set to "1" to use just the major/minor axes

                ellipsoid.setScale(
                    2 * (major * std::sqrt(chi) + obstacle.discs_[0].radius), // std::sqrt(1. / mp),
                    2 * (major * std::sqrt(chi) + obstacle.discs_[0].radius),
                    0.2);

                ellipsoid.addPointMarker(Eigen::Vector3d(collision_regions[index].x, collision_regions[index].y, 0.01));
            }
        }
    }
}