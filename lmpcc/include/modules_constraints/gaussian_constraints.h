/**
 * @file gaussian_constraints.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Gaussian constraints according to https://www.autonomousrobots.nl/docs/19-zhu-RAL.pdf
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __GAUSSIAN_CONSTRAINTS_H__
#define __GAUSSIAN_CONSTRAINTS_H__

#include <lmpcc/types.h>
#include <lmpcc/dynamic_obstacle.h>

#include <ros_tools/ros_visuals.h>

/**
 * @brief Class for using gaussian obstacle avoidance (https://www.autonomousrobots.nl/docs/19-zhu-RAL.pdf).
 * Constraints are based on the inverse error function (see python solver files).
 */
class GaussianConstraints : public ControllerModule
{

public:
    GaussianConstraints(ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle);

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
    void SetParameters(SolverInterface *solver_interface, const RealTimeData &data, int k, int &param_idx) override;

    void Visualize() override;

    void GetMethodName(std::string &name) override { name = "Gaussian"; };

private:
    std::unique_ptr<RosTools::ROSMarkerPublisher> ros_markers_; // For visuals

    std::vector<DynamicObstacle> *obstacles_; // Pointer to obstacle data

    void VisualizeGaussians();
};

#endif // __GAUSSIAN_CONSTRAINTS_H__