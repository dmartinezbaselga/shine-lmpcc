/**
 * @file collision_disc.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Class for modeling collision regions as discs
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef COLLISION_DISC_H
#define COLLISION_DISC_H

#include <Eigen/Dense>
#include <vector>

class VehicleRegion;

/**
 * @brief Models a collision region shaped as a disc. Positions are derived based on offsets and the vehicle position.
 */
struct Disc
{

public:
    int id; // Disc ID for identification

    double radius; // Radius of a disc
    double offset;

    double x;
    double y;

    /**
     * @brief Construct a disc from known parameters
     *
     * @param id ID
     * @param offset Offset
     * @param radius Radius
     */
    Disc(int _id, const Eigen::Vector2d &pos, const double orientation, const double _offset, const double _radius)
        : id(_id), radius(_radius), offset(_offset)
    {
        SetPositionWithVehiclePosition(pos, orientation);
    }

    /** @brief Get the position of this disc as Eigen::Vector2d */
    Eigen::Vector2d AsVector2d() const
    {
        return Eigen::Vector2d(x, y);
    }

    /** @brief Compute the position of this disc based on the given position and orientation of the vehicle */
    void SetPositionWithVehiclePosition(const Eigen::Vector2d &pos, double orientation);

    /** @brief Translate the position of this disc back to the given vehicle center position */
    Eigen::Vector2d TranslateToVehicleCenter(const VehicleRegion &vehicle) const;

    /** @brief Set the position of this disc */
    void SetPosition(const Eigen::Vector2d &new_pos);

    operator std::string() const
    {
        return "disc_" + std::to_string(id);
    }
};

class VehicleRegionTemplate
{
public:
    /**
     * @brief Construct a new Vehicle Region Template object based on precomputed offsets and radius
     *
     * @param n_discs The number of discs
     * @param offsets A vector of the offsets of each disc
     * @param radius The radius of each disc
     *
     * The output should not be used for position or orientation data, but as a template for instanciating other vehicle regions.
     */
    VehicleRegionTemplate(int n_discs, const std::vector<double> &offsets, double radius);
    VehicleRegionTemplate(){};

    double DiscRadius(int id = 0) const { return discs_[id].radius; };

public:
    std::vector<Disc> discs_;

    std::vector<double> offsets_;
};

/** @brief Models a vehicle region. Currently only supports a set of discs @see Disc */
class VehicleRegion : public VehicleRegionTemplate
{
public:
    /** @brief Deprecated: Make a vehicle region using the width and length of the vehicle */
    VehicleRegion(const Eigen::Vector2d &pos, const double orientation, int n_discs, double width, double length, double center_offset); // int n_discs, predictive_configuration *config); // SolverInterface *solver_interface, predictive_configuration *config);
    /** @brief Use a template to create a vehicle region at the given position / orientation */
    VehicleRegion(const Eigen::Vector2d &pos, const double orientation, const VehicleRegionTemplate &vehicle_template); // int n_discs, predictive_configuration *config); // SolverInterface *solver_interface, predictive_configuration *config);

    /** @brief Set the new position of this vehicle. Also updates the disc positions */
    void SetPosition(const Eigen::Vector2d &new_pos);
    void SetPosition(const Eigen::Vector2d &new_pos, const double orientation);

public:
    Eigen::Vector2d pos_;
    double orientation_;
};

#endif