#pragma once

#include <memory>

#include "Obstacle.h"
#include "Path.h"
#include "SwerveDrive.h"
#include "Trajectory.h"

namespace helixtrajectory {
    class TrajectoryGenerator {
    private:
        const SwerveDrive& drive;
        const Path& path;
        const std::vector<Obstacle>& obstacles;

        const size_t waypointCount;
        const size_t trajectorySegmentCount;
        const size_t nPerTrajectorySegment;
        const size_t nTotal;

        casadi::Opti opti;
        casadi::MX X;
        casadi::MX x;
        casadi::MX y;
        casadi::MX theta;
        casadi::MX vx;
        casadi::MX vy;
        casadi::MX omega;
        casadi::MX U;
        casadi::MX ax;
        casadi::MX ay;
        casadi::MX alpha;
        casadi::MX trajectorySegmentTs;
        casadi::MX trajectorySegmentDts;

        /**
         * @brief Apply constraints that represent the fact that the robot is still when starting and
         * ending a path. This could be modified so that the robot is required to have a
         * certain starting and ending velocity.
         * 
         * @param opti the current optimizer
         * @param vx (nTotal + 1) x 1 column vector of velocity x-components for each sample point
         * @param vy (nTotal + 1) x 1 column vector of velocity y-components for each sample point
         * @param omega (nTotal + 1) x 1 column vector of angular velocities for each sample point
         * @param nTotal number of segments in path (number of sample points - 1)
         */
        void ApplyBoundryConstraints();
        void ApplyWaypointConstraints();
    public:
        TrajectoryGenerator(const SwerveDrive& drive, const Path& path, const std::vector<Obstacle>& obstacles);
        std::unique_ptr<Trajectory> Generate();
    };
}