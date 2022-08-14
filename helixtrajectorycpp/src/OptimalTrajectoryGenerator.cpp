#include "OptimalTrajectoryGenerator.h"

#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>

#include <casadi/casadi.hpp>

#include "Drivetrain.h"
#include "HolonomicPath.h"
#include "HolonomicTrajectory.h"
#include "Obstacle.h"
#include "Path.h"
#include "TrajectoryUtil.h"

namespace helixtrajectory {

    OptimalTrajectoryGenerator::OptimalTrajectoryGenerator(const Drivetrain& drivetrain, const Path& path, const std::vector<Obstacle>& obstacles) :
            drivetrain(drivetrain), path(path), obstacles(obstacles),
            waypointCount(path.Length()), trajectorySegmentCount(waypointCount - 1),
            controlIntervalTotal(path.ControlIntervalTotal()),
            trajectorySegmentTs(opti.variable(1, trajectorySegmentCount)),
            trajectorySegmentDts(1, trajectorySegmentCount),
            X(opti.variable(3, controlIntervalTotal + 1)), x(X(0, ALL)), y(X(1, ALL)), theta(X(2, ALL)) {

        std::cout << "Number of Total Control Intervals " << controlIntervalTotal << std::endl;

        XSegments.reserve(waypointCount);
        xSegments.reserve(waypointCount);
        ySegments.reserve(waypointCount);
        thetaSegments.reserve(waypointCount);
        XSegments.push_back(X(ALL, 0));
        XSegments.push_back(x(ALL, 0));
        XSegments.push_back(y(ALL, 0));
        XSegments.push_back(theta(ALL, 0));
        size_t sampleIndex = 0;
        for (size_t waypointIndex = 1; waypointIndex < waypointCount; waypointIndex++) {
            size_t controlIntervalCount = path.GetWaypoint(waypointIndex).controlIntervalCount;
            XSegments.push_back(X(ALL, casadi::Slice((int) sampleIndex, (int) (sampleIndex + controlIntervalCount))));
            xSegments.push_back(x(ALL, casadi::Slice((int) sampleIndex, (int) (sampleIndex + controlIntervalCount))));
            ySegments.push_back(y(ALL, casadi::Slice((int) sampleIndex, (int) (sampleIndex + controlIntervalCount))));
            thetaSegments.push_back(theta(ALL, casadi::Slice((int) sampleIndex, (int) (sampleIndex + controlIntervalCount))));
            sampleIndex += controlIntervalCount;
        }

        casadi::MX totalT = 0;
        for (size_t trajectorySegmentIndex = 0; trajectorySegmentIndex < trajectorySegmentCount; trajectorySegmentIndex++) {
            casadi::MX T = trajectorySegmentTs(trajectorySegmentIndex);
            size_t segmentControlIntervalCount = path.GetWaypoint(trajectorySegmentIndex + 1).controlIntervalCount;
            trajectorySegmentDts(trajectorySegmentIndex) = T / segmentControlIntervalCount;
            totalT += T;
            opti.subject_to(T >= 0);
            opti.set_initial(T, 5);
        }
        std::cout << "Applied Time Constraints" << std::endl;
        opti.minimize(totalT);
        std::cout << "Set Optimization Objective" << std::endl;

        ApplyPathConstraints();
        std::cout << "Applied Path constraints" << std::endl;

        drivetrain.ApplyObstacleConstraints(opti, x, y, theta, controlIntervalTotal, obstacles);
        std::cout << "Applied Obstacle constraints" << std::endl;

        opti.set_initial(X, generateInitialTrajectory(path));
        std::cout << "Set Initial Trajectory" << std::endl;
    }

    OptimalTrajectoryGenerator::~OptimalTrajectoryGenerator() {
    }

    void OptimalTrajectoryGenerator::ApplyPathConstraints() {
        if (path.Length() == 0) {
            return;
        }
        size_t sampleIndex = 0;
        for (int waypointIndex = 0; waypointIndex < waypointCount; waypointIndex++) {
            const Waypoint& waypoint = path.GetWaypoint(waypointIndex);
            sampleIndex += waypoint.controlIntervalCount;
            if (waypoint.xConstrained) {
                opti.subject_to(x(sampleIndex) == waypoint.x);
            }
            if (waypoint.yConstrained) {
                opti.subject_to(y(sampleIndex) == waypoint.y);
            }
            if (waypoint.headingConstrained) {
                // opti.subject_to(fmod(theta(sampleIndex) - waypoint.heading, 2 * M_PI) == 0.0);
                opti.subject_to(theta(sampleIndex) == waypoint.heading);
            }
        }
    }
}