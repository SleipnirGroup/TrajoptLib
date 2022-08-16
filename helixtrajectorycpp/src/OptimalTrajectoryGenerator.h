#pragma once

#include <vector>

#include <casadi/casadi.hpp>

#include "Drivetrain.h"
#include "HolonomicDrivetrain.h"
#include "Path.h"
#include "Obstacle.h"

namespace helixtrajectory {

    /**
     * @brief This class is the superclass for all trajectory generators. It contains the common
     * functionality of all optimizers: waypoint position constraints and obstacle avoidance.
     */
    class OptimalTrajectoryGenerator {
    protected:
        /**
         * @brief the drivetrain
         */
        const Drivetrain& drivetrain;
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
         * @brief the total number of control intervals in the trajectory
         */
        const size_t controlIntervalTotal;
        /**
         * @brief the optimizer
         */
        casadi::Opti opti;

        /**
         * @brief the list of durations of each trajectory segment
         */
        casadi::MX trajectorySegmentTs;
        /**
         * @brief the list of time differentials of each trajectory segment,
         * or the duration of every control interval for each trajectory segment
         */
        casadi::MX trajectorySegmentDts;

        /**
         * @brief The 3 x (controlIntervalTotal + 1) matrix of robot position state per trajectory sample point.
         * Each column is a sample point. The first row is the x-coordinate, the second row is
         * the y-coordinate, and the third row is the heading.
         */
        casadi::MX X;
        /**
         * @brief the 1 x (controlIntervalTotal + 1) vector of the robot's x-coordinate per trajectory sample point
         */
        casadi::MX x;
        /**
         * @brief the 1 x (controlIntervalTotal + 1) vector of the robot's y-coordinate per trajectory sample point
         */
        casadi::MX y;
        /**
         * @brief the 1 x (controlIntervalTotal + 1) vector of the robot's heading per trajectory sample point
         */
        casadi::MX theta;

        std::vector<casadi::MX> XSegments;
        std::vector<casadi::MX> xSegments;
        std::vector<casadi::MX> ySegments;
        std::vector<casadi::MX> thetaSegments;

        /**
         * @brief Construct a new Trajectory Generator from a drivetrain, path, and list of obstacles.
         * 
         * @param drivetrain the drivetrain
         * @param path the path
         * @param obstacles the list of obstacles
         */
        OptimalTrajectoryGenerator(const Drivetrain& drivetrain, const Path& path, const std::vector<Obstacle>& obstacles);

        static const casadi::Slice ALL;

    private:
        /**
         * @brief Applies the constraints that force the robot's motion to comply
         * with the list of waypoints provided. This may include constraints on
         * position and heading.
         */
        static void ApplyPathConstraints(casadi::Opti& opti,
                const std::vector<casadi::MX>& xSegments, const std::vector<casadi::MX>& ySegments,
                const std::vector<casadi::MX>& thetaSegments, const Path& path);
        
        static casadi::DM GenerateInitialGuessX(const Path& path);

    public:
        /**
         * @brief Destroy the Trajectory Generator object
         */
        virtual ~OptimalTrajectoryGenerator();
    };
}