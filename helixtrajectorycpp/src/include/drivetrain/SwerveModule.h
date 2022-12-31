#pragma once

#include <array>
#include <iostream>

#include "constraint/Constraint.h"
// #include "trajectory/HolonomicState.h"

namespace helixtrajectory {

    /**
     * @brief This class represents a single swerve module in a swerve drivetrain.
     * It is defined by the module diagonal, which is the line connecting the origin
     * of the robot coordinate system to the center of the module. The wheel radius,
     * max speed, and max torque must also be specified per module.
     * 
     * @author Justin Babilino
     */
    class SwerveModule {
    public:
        /**
         * @brief x-coordinate of swerve module relative to robot coordinate system
         */
        double x;
        /**
         * @brief y-coordinate of swerve module relative to robot coordinate system
         */
        double y;
        /**
         * @brief radius of wheel
         */
        double wheelRadius;
        /**
         * @brief maximum angular velocity of wheel
         */
        double wheelMaxAngularVelocity;
        /**
         * @brief maximum torque applied to wheel
         */
        double wheelMaxTorque;

        /**
         * @brief Construct a new Swerve Module object with its position, radius, and constraints.
         * 
         * @param x x-coordinate of swerve module
         * @param y y-coordinate of swerve module
         * @param wheelRadius radius of wheel
         * @param wheelMaxAngularVelocity maximum angular velocity of wheel
         * @param wheelMaxTorque maximum torque applied to wheel
         */
        SwerveModule(double x, double y, double wheelRadius, double wheelMaxAngularVelocity, double wheelMaxTorque);

        // std::array<double, 2> CalculateVelocity(double velocityX, double velocityY, double angularVelocity) const noexcept {
        //     std::array<double, 2> velocity;
        //     velocity[0] = velocityX - y * angularVelocity;
        //     velocity[1] = velocityY + x * angularVelocity;
        //     return velocity;
        // }

        /**
         * @brief Append a string representation of a swerve module to an output stream.
         * A string representation of a swerve module is a json object with
         * "x", "y", "wheel_radius", "wheel_max_angular_velocity", and "wheel_max_torque"
         * numerical fields.
         * 
         * @param stream the stream to append the string representation to
         * @param module the swerve module
         * @return a reference to the given stream
         */
        friend std::ostream& operator<<(std::ostream& stream, const SwerveModule& module);
    };
}