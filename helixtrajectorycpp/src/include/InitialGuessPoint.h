#pragma once

#include <iostream>

namespace helixtrajectory {

    /**
     * @brief An initial guess of a possible state the robot may be in during the trajectory.
     */
    class InitialGuessPoint {
    public:
        /**
         * @brief the initial guess of the x-coordinate of the robot
         */
        double x;
        /**
         * @brief the initial guess of the y-coordinate of the robot
         */
        double y;
        /**
         * @brief the initial guess of the heading of the robot
         */
        double heading;

        InitialGuessPoint(double x, double y, double heading);

        friend std::ostream& operator<<(std::ostream& stream, const InitialGuessPoint& guessPoint);
    };
}