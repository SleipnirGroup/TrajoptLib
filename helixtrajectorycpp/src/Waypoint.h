#pragma once

#include <vector>

#include "InitialGuessPoint.h"

namespace helixtrajectory {
    /**
     * @brief A certain state that the robot must have during some instance of the trajectory.
     * Includes options to constrain dynamics like position and velocity during that instance.
     */
    class Waypoint {
    public:
        /**
         * @brief the x-coordinate of robot at waypoint
         */
        double x;
        /**
         * @brief the y-coordinate of robot at waypoint
         */
        double y;
        /**
         * @brief the heading of robot at waypoint
         */
        double heading;
        /**
         * @brief the whether or not the optimizer should constrain the x-coordinate of the robot at waypoint
         */
        bool xConstrained;
        /**
         * @brief whether or not the optimizer should constrain the y-coordinate of the robot at waypoint
         */
        bool yConstrained;
        /**
         * @brief whether or not the optimizer should constrain the heading of the robot at waypoint
         */
        bool headingConstrained;
        /**
         * @brief the number of control intervals in the optimization problem from the last waypoint to this
         * waypoint
         */
        size_t controlIntervalCount;
        /**
         * @brief the points used to construct the linear initial trajectory guess for the trajectory segment
         * from the last waypoint to this waypoint
         */
        std::vector<InitialGuessPoint> initialGuessPoints;

        /**
         * @brief Destroy the Waypoint object
         */
        virtual ~Waypoint();

        /**
         * @brief Checks if this waypoint is valid.
         * 
         * @return true if this waypoint is valid, false otherwise
         */
        virtual bool IsValid() const noexcept;
        /**
         * @brief Checks if the position state at this waypoint is known. This
         * means that the position and heading of the robot is constrained.
         * 
         * @return true if the position state of the robot is constrained, false otherwise
         */
        bool IsPositionStateKnown() const noexcept;
        /**
         * @brief Checks if the velocity state at this waypoint is known. This
         * means that the velocity of the robot is constrained.
         * 
         * @return true if the velocity state of the robot is constrained, false otherwise
         */
        virtual bool IsVelocityStateKnown() const noexcept = 0;
        /**
         * @brief Checks if the positon and velocity at this waypoint is known.
         * 
         * @return true if the position and velocity state of the robot is constrained,
         * false otherwise
         */
        bool IsStateKnown() const noexcept;

    protected:
        /**
         * @brief Construct a new Waypoint object with an x-coordinate, a y-coordinate,
         * a heading, constraint settings, and initial guess points.
         * 
         * @param x the x-coordinate of the robot at this waypoint
         * @param y the y-coordinate of the robot at this waypoint
         * @param heading the heading of the robot at this waypoint
         * @param xConstrained whether or not the optimizer should constrain the x-coordinate of the robot at waypoint
         * @param yConstrained whether or not the optimizer should constrain the y-coordinate of the robot at waypoint
         * @param headingConstrained whether or not the optimizer should constrain the heading of the robot at waypoint
         * @param controlIntervalCount the number of control intervals in the optimization problem from the last waypoint to this
         * waypoint
         * @param initialGuessPoints the points used to construct the linear initial trajectory guess for the trajectory segment
         * from the last waypoint to this waypoint
         */
        Waypoint(double x, double y, double heading,
                bool xConstrained, bool yConstrained, bool headingConstrained,
                size_t controlIntervalCount,
                const std::vector<InitialGuessPoint>& initialGuessPoints);
    };
}