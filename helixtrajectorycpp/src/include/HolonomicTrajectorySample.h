#pragma once

#include <iostream>

#include "TrajectorySample.h"

namespace helixtrajectory {

    /**
     * @brief A sample point cooresponding to the state of the holonomic drivetrain
     * robot at a certain timestamp during the trajectory.
     * 
     * @author Justin Babilino
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
         * @param intervalDuration the duration of the control interval leading up to this sample
         * @param x the x-coordinate of the robot
         * @param y the y-coordinate of the robot
         * @param heading the heading of the robot
         * @param velocityX the x-component of robot velocity
         * @param velocityY the y-component of robot velocity
         * @param angularVelocity the angular velocity of the robot
         */
        HolonomicTrajectorySample(double intervalDuration, double x, double y, double heading,
                double velocityX, double velocityY, double angularVelocity);

        /**
         * @brief Append a string representation of a holonomic trajectory sample to
         * an output stream. A string representation of a holonomic trajectory sample is
         * a json object with the same fields as this object.
         * 
         * @param stream the stream to append the string representation to
         * @param sample the holonomic trajectory sample
         * @return a reference to the given stream
         */
        friend std::ostream& operator<<(std::ostream& stream, const HolonomicTrajectorySample& sample);
    };
}