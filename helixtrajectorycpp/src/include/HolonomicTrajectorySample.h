#pragma once

#include <iostream>

#include "TrajectorySample.h"

namespace helixtrajectory {

    /**
     * @brief A sample point cooresponding to the state of the holonomic drivetrain
     * robot at a certain timestamp during the trajectory.
     */
    class HolonomicTrajectorySample : public TrajectorySample {
    public:
        /**
         * @brief the x-component of robot velocity
         */
        double velocityX;
        /**
         * @brief the y-component of robot velocity
         */
        double velocityY;
        /**
         * @brief the angular velocity of the robot
         */
        double angularVelocity;

        /**
         * @brief Construct a new Holonomic Trajectory Sample object with its position
         * and velocity state and timestamp.
         * 
         * @param timestamp the timestamp of this sample
         * @param x the x-coordinate of the robot
         * @param y the y-coordinate of the robot
         * @param heading the heading of the robot
         * @param velocityX the x-component of robot velocity
         * @param velocityY the y-component of robot velocity
         * @param angularVelocity the angular velocity of the robot
         */
        HolonomicTrajectorySample(double x, double y, double heading,
                double velocityX, double velocityY, double angularVelocity);

        friend std::ostream& operator<<(std::ostream& stream, const HolonomicTrajectorySample& sample);
    };
}