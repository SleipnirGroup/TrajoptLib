#pragma once

#include <vector>

#include <casadi/casadi.hpp>

#include "Drive.h"
#include "HolonomicDrive.h"
#include "Path.h"
#include "Obstacle.h"

namespace helixtrajectory {

    /**
     * @brief This class is the superclass for all trajectory generators. It contains the common
     * functionality of all optimizers: waypoint position constraints and obstacle avoidance.
     */
    class TrajectoryGenerator {
    protected:
        /**
         * @brief the drive
         */
        const Drive& drive;
        /**
         * @brief the path
         */
        const Path& path;
        /**
         * @brief the list of obstacles
         */
        const std::vector<Obstacle>& obstacles;

        /**
         * @brief the number of waypoints in the path
         */
        const size_t waypointCount;
        /**
         * @brief the number of trajectory segments in the trajectory
         */
        const size_t trajectorySegmentCount;
        /**
         * @brief the number of segments in a trajectory segment
         */
        const size_t nPerTrajectorySegment;
        /**
         * @brief the total number of segments in the trajectory
         */
        const size_t nTotal;

        /**
         * @brief the optimizer
         */
        casadi::Opti opti;

        /**
         * @brief the list of durations of each trajectory segment
         */
        casadi::MX trajectorySegmentTs;
        /**
         * @brief the list of time differentials of each trajectory segment
         */
        casadi::MX trajectorySegmentDts;

        /**
         * @brief The 3 x (nTotal + 1) matrix of robot position state per trajectory sample point.
         * Each column is a sample point. The first row is the x-coordinate, the second row is
         * the y-coordinate, and the third row is the heading.
         */
        casadi::MX X;
        /**
         * @brief the 1 x (nTotal + 1) vector of the robot's x-coordinate per trajectory sample point
         */
        casadi::MX x;
        /**
         * @brief the 1 x (nTotal + 1) vector of the robot's y-coordinate per trajectory sample point
         */
        casadi::MX y;
        /**
         * @brief the 1 x (nTotal + 1) vector of the robot's heading per trajectory sample point
         */
        casadi::MX theta;

        /**
         * @brief Construct a new Trajectory Generator from a drive, path, and list of obstacles.
         * 
         * @param drive the drive
         * @param path the path
         * @param obstacles the list of obstacles
         */
        TrajectoryGenerator(const Drive& drive, const Path& path, const std::vector<Obstacle>& obstacles);

    private:
        /**
         * @brief Applies the constraints that force the robot's motion to comply
         * with the list of waypoints provided. This may include constraints on
         * position and heading.
         */
        void ApplyPathConstraints();

    public:
        virtual ~TrajectoryGenerator();
    };
}