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

namespace helixtrajectory {

    const casadi::Slice OptimalTrajectoryGenerator::ALL = casadi::Slice();

    OptimalTrajectoryGenerator::OptimalTrajectoryGenerator(const Drivetrain& drivetrain, const Path& path, const std::vector<Obstacle>& obstacles) :
            drivetrain(drivetrain), path(path), obstacles(obstacles),
            waypointCount(path.Length()), trajectorySegmentCount(waypointCount - 1),
            controlIntervalTotal(path.ControlIntervalTotal()),
            trajectorySegmentTs(opti.variable(1, trajectorySegmentCount)),
            trajectorySegmentDts(1, trajectorySegmentCount),
            X(opti.variable(3, controlIntervalTotal + 1)), x(X(0, ALL)), y(X(1, ALL)), theta(X(2, ALL)) {

        if (!path.IsValid()) {
            throw "path is not valid";
        }

        std::cout << "Number of Total Control Intervals " << controlIntervalTotal << std::endl;

        XSegments.reserve(waypointCount);
        xSegments.reserve(waypointCount);
        ySegments.reserve(waypointCount);
        thetaSegments.reserve(waypointCount);
        XSegments.push_back(X(ALL, 0));
        xSegments.push_back(x(ALL, 0));
        ySegments.push_back(y(ALL, 0));
        thetaSegments.push_back(theta(ALL, 0));
        size_t sampleIndex = 1;
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

        ApplyPathConstraints(opti, xSegments, ySegments, thetaSegments, OptimalTrajectoryGenerator::path);
        std::cout << "Applied Path constraints" << std::endl;

        drivetrain.ApplyObstacleConstraints(opti, x, y, theta, controlIntervalTotal, OptimalTrajectoryGenerator::obstacles);
        std::cout << "Applied Obstacle constraints" << std::endl;

        opti.set_initial(X, GenerateInitialGuessX(path));
        std::cout << "Set Initial Trajectory" << std::endl;
    }

    OptimalTrajectoryGenerator::~OptimalTrajectoryGenerator() {
    }

    void OptimalTrajectoryGenerator::ApplyPathConstraints(casadi::Opti& opti,
                const std::vector<casadi::MX>& xSegments, const std::vector<casadi::MX>& ySegments,
                const std::vector<casadi::MX>& thetaSegments, const Path& path) {
        for (int waypointIndex = 0; waypointIndex < path.Length(); waypointIndex++) {
            const Waypoint& waypoint = path.GetWaypoint(waypointIndex);
            if (waypoint.xConstrained) {
                opti.subject_to(xSegments[waypointIndex](-1) == waypoint.x);
            }
            if (waypoint.yConstrained) {
                opti.subject_to(ySegments[waypointIndex](-1) == waypoint.y);
            }
            if (waypoint.headingConstrained) {
                // opti.subject_to(fmod(thetaSegments[waypointIndex](-1) - waypoint.heading, 2 * M_PI) == 0.0);
                opti.subject_to(thetaSegments[waypointIndex](-1) == waypoint.heading);
            }
        }
    }

    void linspace(casadi::DM& arry, size_t startIndex, size_t endIndex, double startValue, double endValue) {
        size_t segmentCount = endIndex - startIndex;
        double delta = (endValue - startValue) / segmentCount;
        for (int index = 0; index < segmentCount; index++) {
            arry(startIndex + index) = startValue + index * delta;
        }
    }

    casadi::DM OptimalTrajectoryGenerator::GenerateInitialGuessX(const Path& path) {
        size_t waypointCount = path.Length();
        casadi::DM X(3, path.ControlIntervalTotal() + 1);
        casadi::DM x = X(0, ALL);
        casadi::DM y = X(1, ALL);
        casadi::DM theta = X(2, ALL);
        size_t sampleIndex = path.GetWaypoint(0).controlIntervalCount;
        for (size_t waypointIndex = 1; waypointIndex < waypointCount; waypointIndex++) {
            const Waypoint& previousWaypoint = path.GetWaypoint(waypointIndex - 1);
            const Waypoint& waypoint = path.GetWaypoint(waypointIndex);
            size_t intervalCount = waypoint.controlIntervalCount;
            size_t guessPointCount = waypoint.initialGuessPoints.size();
            size_t previousWaypointSampleIndex = sampleIndex;
            size_t waypointSampleIndex = previousWaypointSampleIndex + intervalCount;
            if (guessPointCount == 0) {
                linspace(x, previousWaypointSampleIndex, waypointSampleIndex, previousWaypoint.x, waypoint.x);
                linspace(y, previousWaypointSampleIndex, waypointSampleIndex, previousWaypoint.y, waypoint.y);
                linspace(theta, previousWaypointSampleIndex, waypointSampleIndex, previousWaypoint.heading, waypoint.heading);
            } else {
                size_t guessSegmentIntervalCount = intervalCount / (guessPointCount + 1);
                linspace(x, previousWaypointSampleIndex, previousWaypointSampleIndex + guessSegmentIntervalCount, previousWaypoint.x, waypoint.initialGuessPoints[0].x);
                linspace(y, previousWaypointSampleIndex, previousWaypointSampleIndex + guessSegmentIntervalCount, previousWaypoint.y, waypoint.initialGuessPoints[0].y);
                linspace(theta, previousWaypointSampleIndex, previousWaypointSampleIndex + guessSegmentIntervalCount, previousWaypoint.heading, waypoint.initialGuessPoints[0].heading);
                size_t firstGuessPointSampleIndex = previousWaypointSampleIndex + guessSegmentIntervalCount;
                for (int guessPointIndex = 1; guessPointIndex < guessPointCount; guessPointIndex++) {
                    size_t previousGuessPointSampleIndex = firstGuessPointSampleIndex + (guessPointIndex - 1) * guessSegmentIntervalCount;
                    size_t guessPointSampleIndex = firstGuessPointSampleIndex + (guessPointIndex) * guessSegmentIntervalCount;
                    linspace(x, previousGuessPointSampleIndex, guessPointSampleIndex, waypoint.initialGuessPoints[guessPointIndex - 1].x, waypoint.initialGuessPoints[guessPointIndex].x);
                    linspace(y, previousGuessPointSampleIndex, guessPointSampleIndex, waypoint.initialGuessPoints[guessPointIndex - 1].y, waypoint.initialGuessPoints[guessPointIndex].y);
                    linspace(theta, previousGuessPointSampleIndex, guessPointSampleIndex, waypoint.initialGuessPoints[guessPointIndex - 1].heading, waypoint.initialGuessPoints[guessPointIndex].heading);
                }
                size_t finalGuessPointSampleIndex = previousWaypointSampleIndex + guessPointCount * guessSegmentIntervalCount;
                linspace(x, finalGuessPointSampleIndex, waypointSampleIndex, waypoint.initialGuessPoints[guessPointCount - 1].x, waypoint.x);
                linspace(y, finalGuessPointSampleIndex, waypointSampleIndex, waypoint.initialGuessPoints[guessPointCount - 1].y, waypoint.y);
                linspace(theta, finalGuessPointSampleIndex, waypointSampleIndex, waypoint.initialGuessPoints[guessPointCount - 1].heading, waypoint.heading);
            }
            sampleIndex += intervalCount;
        }
        x(sampleIndex) = path.GetWaypoint(waypointCount - 1).x;
        y(sampleIndex) = path.GetWaypoint(waypointCount - 1).y;
        theta(sampleIndex) = path.GetWaypoint(waypointCount - 1).heading;

        return X;
    }
}