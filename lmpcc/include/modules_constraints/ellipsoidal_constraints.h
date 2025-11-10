/**
 * @file ellipsoid_constraints.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Class for inequality constraints based on ellipsoidal level sets of a Gaussian distribution
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef ELLIPSOID_CONSTRAINTS_H
#define ELLIPSOID_CONSTRAINTS_H

#include <lmpcc/types.h>
#include <lmpcc/dynamic_obstacle.h>

#include <ros_tools/ros_visuals.h>
// #include "ros_tools/helpers.h"

#include <vector>

/**
 * @brief Class for using ellipsoidal obstacle avoidance. Makes use of config_->risk_ to size the ellipsoids to a probability.
 *
 */
class EllipsoidalConstraints : public ControllerModule
{

public:
    EllipsoidalConstraints(ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle);

public:
    /**
     * @brief Update the ellipsoid data
     *
     * @param dynamic_obstacles
     */
    void Update(SolverInterface *solver_interface, RealTimeData &data);

    /**
     * @brief Insert the collision avoidance constraints
     *
     * @param solver_interface The solver
     * @param k The optimization stage
     * @param param_idx The parameter to insert into (is updated)
     */
    void SetParameters(SolverInterface *solver_interface, const RealTimeData &data, int k_solver, int &param_idx);

    /**
     * @brief Visualize the ellipsoidal obstacles
     *
     */
    void Visualize();

    void GetMethodName(std::string &name) override
    {
        if (name == "")
        {
            name = "LMPCC"; // Only overwrite if no method name was given yet
        }
    };

private:
    std::unique_ptr<RosTools::ROSMarkerPublisher> ros_markers_; // For visuals

    std::vector<DynamicObstacle> *obstacles_; // Pointer to obstacle data

    void VisualizeEllipsoids();
};

#endif