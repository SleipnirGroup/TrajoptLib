#pragma once

#include <vector>

#include <casadi/casadi.hpp>

#include "HolonomicDrive.h"
#include "Obstacle.h"

namespace helixtrajectory {

    struct SwerveModule {
        /**
         * @brief the x-coordinate of the swerve module relative to the robot coordinate system
         */
        double x;
        /**
         * @brief the y-coordinate of the swerve module relative to the robot coordinate system
         */
        double y;
        /**
         * @brief radius of wheels
         */
        double wheelRadius;
        /**
         * @brief maximum angular velocity of wheels
         */
        double wheelMaxAngularVelocity;
        /**
         * @brief maximum torque applied to wheels
         */
        double wheelMaxTorque;

        double GetModuleDiagonal() const {
            return hypot(x, y);
        }
        double GetModuleAngle() const {
            return atan2(y, x);
        }
    };
    /**
     * @brief This class represents a swerve drive robot. It includes the physical properties necessary to
     * accurately model the dynamics of the system. To understand this code, some definitions
     * are required.
     * 
     * When properties for each module (or bumper corner) are listed, they are always in the order of
     * [front_left, front_right, rear_left, rear_right].
     * Related to the 2D coordinate system of the field, the robot coordinate system is defined with its
     * center placed on the center of the robot (which is the center of a rectangle with corners touching
     * each swerve module), and the x-axis points directly towards the front face of the robot. The y-axis
     * points 90 degress counter-clockwise. The following diagram shows the coordinate system and some
     * related dimensions:
     *  _________________________________________                    \ 
     * /                                         \                   |
     * |   00                              00    |                   |
     * |   00  rear_left        front_left  00   |   \               |
     * |   00               y                00  |   |               |
     * |                    ^                    |   | wheelbase_y   |
     * |                    |                    |   |               |
     * |                    +----> x             |   /               | width
     * |                                         |                   |
     * |                                         |                   |
     * |  00                                00   |                   |
     * |   00  rear_right      front_right  00   |                   |
     * |    00                              00   |                   |
     * |                                         |                   |
     * \_________________________________________/                   /
     *                      <- wheelbase_x ->
     * <----------------- length ---------------->
     * 
     * The nonrotating robot coordinate system is defined with the same center as the robot coordinate
     * system, but it does not rotate and its axes always point in the same directions as the field
     * coordinate system. The robot rotates relative to the nonrotating robot coordinate system.
     */
    class SwerveDrive : public HolonomicDrive {
    public:
        /**
         * @brief the list of swerve modules that make the robot move, usually one in each corner
         */
        std::vector<SwerveModule> modules;

        /**
         * @brief Construct a SwerveDrive object with the appropriate properties.
         * 
         * @param wheelbaseX x coordinate of the front or rear right module relative to the nonrotating robot coordinate system
         * @param wheelbaseY y coordinate of the front or rear right module relative to the nonrotating robot coordinate system
         * @param length bumper-to-bumper distance in x-direction relative to the nonrotating robot coordinate system
         * @param width bumper-to-bumper distance in y-direction relative to the nonrotating robot coordinate system
         * @param mass mass of the robot
         * @param moi moment of inertia of robot about axis of rotation (currently through
         *            center of robot coordinate system)
         * @param wheelMaxAngularVelocity maximum angular velocity of wheels 
         * @param wheelMaxTorque maximum torque applied to wheels
         * @param wheelRadius radius of wheels
         */
        SwerveDrive(double wheelbaseX, double wheelbaseY, double mass, double moi,
                double wheelMaxAngularVelocity, double wheelMaxTorque, double wheelRadius, const Obstacle& bumpers);
        virtual void ApplyKinematicsConstraints(casadi::Opti& opti,
                const casadi::MX& theta, const casadi::MX& vx, const casadi::MX& vy,
                const casadi::MX& omega, const casadi::MX& ax, const casadi::MX& ay,
                const casadi::MX& alpha, const size_t nTotal) const;

    private:
        /**
         * @brief Gives expressions for the position of each module relative to the nonrotating robot
         * coordinate system with a certain heading. Returns a 2 x 4 matrix of position expressions.
         * Each column is a certain module, with the module order specified in this class. The first row
         * contains the x-coordinates, and the second row contains the y-coordinates.
         * 
         * @param theta the instantaneous heading of the robot
         * @return a 2 x 4 matrix of positions where each column is a module and each row is a coordinate
         */
        const casadi::MX SolveModulePosition(const casadi::MX& theta, const SwerveModule& module) const;
    };
}