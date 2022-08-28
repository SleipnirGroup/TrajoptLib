#pragma once

#include <iostream>
#include <math.h>

namespace helixtrajectory {

    /**
     * @brief This struct represents a single swerve module in a swerve drivetrain.
     * It is defined by the module diagonal, which is the line connecting the origin
     * of the robot coordinate system to the center of the module. The wheel radius,
     * max speed, and max torque must also be specified per module.
     * 
     * @author Justin Babilino
     */
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

        /**
         * @brief Calculates the length of the module diagonal, which is the 
         * distance from the origin of the robot coordinate system to the
         * center of the module.
         * 
         * @return the module diagonal length
         */
        double GetModuleDiagonal() const {
            return hypot(x, y);
        }
        /**
         * @brief Calculates the angle between the x-axis of the robot coordinate
         * system and the module diagonal
         * 
         * @return the module diagonal angle
         */
        double GetModuleAngle() const {
            return atan2(y, x);
        }

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