/**
 * @file linearized_constraints.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Deterministic dynamic constraints linearized with respect to the vehicle plan
 * @version 0.1
 * @date 2022-07-27
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef LINEARIZED_CONSTRAINTS_H
#define LINEARIZED_CONSTRAINTS_H

#include <lmpcc/types.h>
#include <lmpcc/dynamic_obstacle.h>

#include <ros_tools/ros_visuals.h>

/**
 * @brief Class for using ellipsoidal obstacle avoidance. Makes use of config_->risk_ to size the ellipsoids to a probability.
 *
 */
class LinearizedConstraints : public ControllerModule
{

public:
    LinearizedConstraints(ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle);

public:
    /**
     * @brief Update the ellipsoid data
     *
     * @param dynamic_obstacles
     */
    void Update(SolverInterface *solver_interface, RealTimeData &data) override;

    /**
     * @brief Insert the collision avoidance constraints
     *
     * @param solver_interface The solver
     * @param k The optimization stage
     * @param param_idx The parameter to insert into (is updated)
     */
    void SetParameters(SolverInterface *solver_interface, const RealTimeData &data, int k_solver, int &param_idx) override;

    /**
     * @brief Visualize the ellipsoidal obstacles
     *
     */
    void Visualize();

    void GetMethodName(std::string &name) override { name = "lin-mpcc"; };

    int NumActiveConstraints(SolverInterface *solver_interface);

private:
    std::unique_ptr<RosTools::ROSMarkerPublisher> ros_markers_, ros_static_markers_; // For visuals

    std::vector<DynamicObstacle> *obstacles_; // Pointer to obstacle data
    RealTimeData *data_ptr_ = nullptr;

    std::vector<std::vector<Eigen::ArrayXd>> a1_, a2_, b_; // Constraints [disc x step]

    RosTools::DouglasRachford dr_projection_;

    int num_obstacles_;

    void ProjectToSafety(int k, Eigen::Vector2d &pos, std::vector<DynamicObstacle> &obstacles, double vehicle_radius);
};

#endif