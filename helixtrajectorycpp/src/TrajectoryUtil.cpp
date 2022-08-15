#include <iostream>
#include <array>

#include "TrajectoryUtil.h"

namespace helixtrajectory {

    void linspace(casadi::DM& arry, size_t startIndex, size_t endIndex, double startValue, double endValue) {
        size_t segmentCount = endIndex - startIndex;
        double delta = (endValue - startValue) / segmentCount;
        for (int index = 0; index < segmentCount; index++) {
            arry(startIndex + index) = startValue + index * delta;
        }
    }

    casadi::DM generateInitialTrajectory(const Path& path) {
        // if (path.Length() == 0) {
        //     return;
        // }
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

    void printHolonomicPath(const HolonomicPath& path) {
        std::cout << "[\n";
        for (const HolonomicWaypoint& waypoint : path.holonomicWaypoints) {
            std::cout << "    {\n";
            std::cout << "        \"x\": " << waypoint.x << ",\n";
            std::cout << "        \"y\": " << waypoint.y << ",\n";
            std::cout << "        \"heading\": " << waypoint.heading << "\n";
            std::cout << "    },\n";
        }
        std::cout << "]" << std::endl;
    }
}