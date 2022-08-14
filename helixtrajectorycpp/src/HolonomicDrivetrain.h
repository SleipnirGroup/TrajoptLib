#pragma once

#include <casadi/casadi.hpp>

#include "Drivetrain.h"
#include "Obstacle.h"

namespace helixtrajectory {

    /**
     * @brief Represents a type of drivetrain that is holonomic. Holonomic drivetrains allow
     * the robot to have complete (or approximate) control over the three degrees of freedom:
     * position and rotation. For example, mecanum drivetrains and swerve drivetrains can both
     * manipulate their motors to have any overall velocity vector while having any heading.
     * HolonomicDrivetrain introduces no new fields, but it forces subtypes to implement the
     * kinematics constraints method, which ties the robot's overall velocity variables to the
     * actuators on the drivetrain.
     */
    class HolonomicDrivetrain : public Drivetrain {
    protected:
        /**
         * @brief Construct a new HolonomicDrivetrain with the robot's mass, moment of inertia, and bumpers.
         * 
         * @param mass the mass of the entire robot
         * @param momentOfInertia the moment of inertia of the robot about the center of rotation, which 
         * @param bumpers the bumpers of the robot represented as an obstacle
         */
        HolonomicDrivetrain(double mass, double momentOfInertia, const Obstacle& bumpers);

    public:

        /**
         * @brief Destroy the Holonomic Drivetrain object
         */
        virtual ~HolonomicDrivetrain();

        /**
         * @brief Applies the drivetrain-specific constraints to the optimizer. These constraints
         * prevent motors from spinning too fast or with too much power. 
         * 
         * @param opti the current optimizer upon which constraints will be applied
         * @param theta 1 x (controlIntervalTotal + 1) vector of the robot's heading for each sample point
         * @param vx 1 x (controlIntervalTotal + 1) vector of the x-coordinate of the robot's velocity for each sample point
         * @param vy 1 x (controlIntervalTotal + 1) vector of the y-coordinate of the robot's velocity for each sample point
         * @param omega 1 x (controlIntervalTotal + 1) vector of the robot's angular velocity for each sample point
         * @param ax 1 x (controlIntervalTotal) vector of the x-coordinate of the robot's acceleration for each sample
         *           point
         * @param ay 1 x (controlIntervalTotal) vector of the y-coordinate of the robot's acceleration for each sample
         *           point
         * @param alpha 1 x (controlIntervalTotal) vector of the robot's angular velocity for each sample point
         * @param controlIntervalTotal the number of segments in this trajectory (number of sample points - 1)
         */
        virtual void ApplyKinematicsConstraints(casadi::Opti& opti,
                const casadi::MX& theta, const casadi::MX& vx, const casadi::MX& vy,
                const casadi::MX& omega, const casadi::MX& ax, const casadi::MX& ay,
                const casadi::MX& alpha, size_t controlIntervalTotal) const = 0;
    };
}