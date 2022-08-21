#pragma once

#include <iostream>
#include <vector>

#include <casadi/casadi.hpp>

#include "Obstacle.h"

namespace helixtrajectory {

    /**
     * @brief This class represents an abstract robot with bumpers. The underlying robot
     * drivetrain could be differential, swerve, mecanum, omni, etc. This class provides
     * the common obstacle avoidance constraints and mass and moment of inertia fields.
     * Related to the 2D coordinate system of the field called the field coordinate system,
     * the robot coordinate system is defined with its center as the center of the robot,
     * and it rotates as the robot rotates. The nonrotating robot coordinate system is defined
     * with the same center as the robot coordinate system, but it does not rotate and its axes
     * always point in the same directions as the field coordinate system. The robot rotates
     * relative to the nonrotating robot coordinate system.
     * 
     * The axis of rotation of the robot pierces through the origin of the robot coordinate
     * system, normal to the plane of the field.
     * 
     * Bumpers are represented as an obstacle. A bumper corner is analogous to an obstacle point
     * on the bumpers. The bumper corner points are specified relative to the robot coordinate
     * system. Using an obstacle as the robot's bumpers is very powerful, as it is possible to create
     * bumpers with rounded corners, circular bumpers, rectanglar bumpers with two semi circles
     * on side, or many other shapes.
     */
    class Drivetrain {
    private:
        /**
         * @brief Gives an expression for the position of a bumper corner relative
         * to the field coordinate system, given the robot's x-coordinate, y-coordinate,
         * and heading. The first row contains the x-coordinate, and the second row
         * contains the y-coordinate.
         * 
         * @param x the instantaneous heading of the robot (scalar)
         * @param y the instantaneous heading of the robot (scalar)
         * @param theta the instantaneous heading of the robot (scalar)
         * @param bumperCorner the bumper corner to find the position for
         * @return a 2 x 1 vector of positions where each row is a coordinate
         */
        const casadi::MX SolveBumperCornerPosition(const casadi::MX& x, const casadi::MX& y,
                const casadi::MX& theta, const ObstaclePoint& bumperCorner) const;
    protected:
        /**
         * @brief Construct a new Drive object with the robot's mass, moment of inertia,
         * and bumpers. Bumpers are represented as an obstacle.
         * 
         * @param mass the mass of the entire robot
         * @param momentOfInertia the moment of inertia of the robot about the center of rotation
         * @param bumpers the bumpers of the robot represented as an obstacle
         */
        Drivetrain(double mass, double momentOfInertia, const Obstacle& bumpers);

    public:
        /**
         * @brief mass of the robot
         */
        double mass;
        /**
         * @brief moment of inertia of robot about axis of rotation, through
         * center of robot coordinate system
         */
        double momentOfInertia;
        /**
         * @brief the boundaries of the robot's bumpers, represented as an Obstacle.
         */
        Obstacle bumpers;

        /**
         * @brief Destroy the Drivetrain object
         */
        virtual ~Drivetrain();

        /**
         * @brief Applies constraints that prevent the robot from getting too close to a given obstacle.
         * If the robot's bumpers and an obstacle are made from a single point, a minimum distance constraint
         * is applied. Otherwise, constraints that prevent a point on the bumpers from getting too close to
         * an obstacle line segment and prevent a line segment on the bumpers from getting too close to an obstacle
         * point are created.
         * 
         * @param opti the current optimizer
         * @param x 1 x (controlIntervalTotal + 1) vector of the x-coordinate of the robot for each sample point
         * @param y 1 x (controlIntervalTotal + 1) vector of the y-coordinate of the robot for each sample point
         * @param theta 1 x (controlIntervalTotal + 1) column vector of the robot's heading for each sample point
         * @param controlIntervalTotal the number of control intervals in this trajectory (number of sample points - 1)
         * @param obstacles the list of obstacles to apply constraints for
         */
        void ApplyObstacleConstraints(casadi::Opti& opti, const casadi::MX& x, const casadi::MX& y,
                const casadi::MX& theta, size_t controlIntervalTotal, const std::vector<Obstacle>& obstacles) const;
    };
}