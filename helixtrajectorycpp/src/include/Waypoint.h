#pragma once

#include <vector>

#include "InitialGuessPoint.h"
#include "Obstacle.h"

namespace helixtrajectory {

    /**
     * @brief A certain state that the robot must have during some instance of the trajectory.
     * Includes options to constrain dynamics like position and velocity during that instance.
     * 
     * @author Justin Babilino
     */
    class Waypoint {
    public:
        /**
         * @brief the x-coordinate of the robot at this waypoint
         */
        double x;
        /**
         * @brief the y-coordinate of the robot at this waypoint
         */
        double y;
        /**
         * @brief the heading of the robot at this waypoint
         */
        double heading;
        /**
         * @brief whether or not the optimizer should constrain the x-coordinate of the robot at this waypoint
         */
        bool xConstrained;
        /**
         * @brief whether or not the optimizer should constrain the y-coordinate of the robot at this waypoint
         */
        bool yConstrained;
        /**
         * @brief whether or not the optimizer should constrain the heading of the robot at this waypoint
         */
        bool headingConstrained;
        /**
         * @brief the number of control intervals in the optimization problem from the previous waypoint to this
         * waypoint
         */
        size_t controlIntervalCount;
        /**
         * @brief the points used to construct the linear initial trajectory guess for the trajectory segment
         * from the last waypoint to this waypoint
         */
        std::vector<InitialGuessPoint> initialGuessPoints;
        /**
         * @brief the collection of obstacles that the robot must avoid while approaching this waypoint
         */
        std::vector<Obstacle> obstacles;

        /**
         * @brief Destroy the Waypoint object
         */
        virtual ~Waypoint() = default;

        /**
         * @brief Check if this waypoint is valid.
         * 
         * @return true if this waypoint is valid, false otherwise
         */
        virtual bool IsValid() const noexcept;
        /**
         * @brief Check if the position state at this waypoint is known. This
         * means that the position and heading of the robot is constrained.
         * 
         * @return true if the position state of the robot is constrained, false otherwise
         */
        bool IsPositionStateKnown() const noexcept;
        /**
         * @brief Check if the velocity state at this waypoint is known. This
         * means that the velocity of the robot is constrained.
         * 
         * @return true if the velocity state of the robot is constrained, false otherwise
         */
        virtual bool IsVelocityStateKnown() const noexcept = 0;
        /**
         * @brief Check if the positon and velocity at this waypoint is known.
         * 
         * @return true if the position and velocity state of the robot is constrained,
         * false otherwise
         */
        bool IsStateKnown() const noexcept;

    protected:
        /**
         * @brief Construct a new Waypoint object with its position state, position constraint settings,
         * control interval count, and initial guess points.
         * 
         * @param x the x-coordinate of the robot at this waypoint
         * @param y the y-coordinate of the robot at this waypoint
         * @param heading the heading of the robot at this waypoint
         * @param xConstrained whether or not the optimizer should constrain the x-coordinate of the robot at this waypoint
         * @param yConstrained whether or not the optimizer should constrain the y-coordinate of the robot at this waypoint
         * @param headingConstrained whether or not the optimizer should constrain the heading of the robot at this waypoint
         * @param controlIntervalCount the number of control intervals in the optimization problem from the previous waypoint to this
         * waypoint
         * @param initialGuessPoints the points used to construct the linear initial trajectory guess for the trajectory segment
         * from the last waypoint to this waypoint
         * @param obstacles the collection of obstacles that the robot must avoid while approaching this waypoint
         */
        Waypoint(double x, double y, double heading,
                bool xConstrained, bool yConstrained, bool headingConstrained,
                size_t controlIntervalCount,
                const std::vector<InitialGuessPoint>& initialGuessPoints,
                const std::vector<Obstacle>& obstacles);
    };
}