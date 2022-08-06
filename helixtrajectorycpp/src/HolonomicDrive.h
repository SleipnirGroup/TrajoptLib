#pragma once

#include <casadi/casadi.hpp>

#include "Drive.h"
#include "Obstacle.h"

namespace helixtrajectory {

    /**
     * @brief Represents a type of drivetrain that is holonomic. Holonomic drivetrains allow
     * the robot to have complete (or approximate) control over the three degrees of freedom:
     * position and rotation. For example, mecanum drivetrains and swerve drivetrains can both
     * manipulate their motors to have any overall velocity vector while having any heading.
     * HolonomicDrive introduces no new fields, but it forces subtypes to implement the
     * kinematics constraints method.
     */
    class HolonomicDrive : public Drive {
    protected:
        /**
         * @brief Construct a new HolonomicDrive with the robot's mass, moment of inertia, and bumpers.
         * 
         * @param mass the mass of the entire robot
         * @param moi the moment of inertia of the robot about the center of rotation, which 
         * @param bumpers the bumpers of the robot represented as an obstacle
         */
        HolonomicDrive(double mass, double moi, const Obstacle& bumpers);

    public:

        virtual ~HolonomicDrive();

        /**
         * @brief Applies the drivetrain-specific constraints to the optimizer. These constraints
         * prevent motors from spinning too fast or with too much power. 
         * 
         * @param opti the current optimizer upon which constraints will be applied
         * @param theta (nTotal + 1) x 1 column vector of the robot's heading for each sample point
         * @param vx (nTotal + 1) x 1 column vector of the x-coordinate of the robot's velocity for each sample point
         * @param vy (nTotal + 1) x 1 column vector of the y-coordinate of the robot's velocity for each sample point
         * @param omega (nTotal + 1) x 1 column vector of the robot's angular velocity for each sample point
         * @param ax nTotal x 1 column vector of the x-coordinate of the robot's acceleration for each sample
         *           point
         * @param ay nTotal x 1 column vector of the y-coordinate of the robot's acceleration for each sample
         *           point
         * @param alpha nTotal x 1 column vector of the robot's angular velocity for each sample point
         * @param nTotal the number of segments in this trajectory (number of sample points - 1)
         */
        virtual void ApplyKinematicsConstraints(casadi::Opti& opti,
                const casadi::MX& theta, const casadi::MX& vx, const casadi::MX& vy,
                const casadi::MX& omega, const casadi::MX& ax, const casadi::MX& ay,
                const casadi::MX& alpha, size_t nTotal) const = 0;
    };
}