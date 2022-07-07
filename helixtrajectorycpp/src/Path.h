#pragma once

#include <vector>

namespace helixtrajectory {

    struct InitialGuessPoint {
        double x;
        double y;
    };
    /**
     * @brief A certain state that the robot must have during some instance of the trajectory.
     * Includes options to constrain dynamics like position and velocity during that instance.
     */
    struct Waypoint {
        /**
         * @brief x-coordinate of robot at waypoint
         */
        double x;
        /**
         * @brief y-coordinate of robot at waypoint
         */
        double y;
        /**
         * @brief heading of robot at waypoint
         */
        double heading;
        /**
         * @brief x-coordinate of robot velocity at waypoint
         */
        double vx;
        /**
         * @brief y-coordinate of robot velocity at waypoint
         */
        double vy;
        /**
         * @brief angular velocity of robot at waypoint
         */
        double omega;
        /**
         * @brief indicates which parts of the robot state must be constrained, and which parts are
         * free to be selected by the optimizer.
         */
        bool xConstrained, yConstrained, headingConstrained,
                vxConstrained, vyConstrained, vMagnitudeConstrained, omegaConstrained;
        /**
         * @brief the points used to construct the initial trajectory guess for the next trajectory segment
         */
        std::vector<InitialGuessPoint> initialGuessPoints;
    };

    /**
     * @brief A sequence of waypoints that make up a path that the robot can follow. Note that
     * and a Trajectory is the detailed output of the optimizer that tells the robot exactly how
     * to move.
     */
    struct Path {
        /**
         * @brief the waypoints that make up this path
         */
        std::vector<Waypoint> waypoints;
        /**
         * @brief Construct a Path with a list of waypoints
         * 
         * @param waypoints the waypoints that make up this path
         */
        Path(const std::vector<Waypoint>& waypoints);
    };
}