#include <iostream>
#include <array>

#include "TrajectoryUtil.h"

namespace helixtrajectory {

    void linspace(casadi::DM& x, size_t row, double start, double end, double n) {
        double delta = (end - start) / n;
        for (int i = 0; i < n; i++) {
            x(row, i) = start + i * delta;
        }
    }

    casadi::DM generateInitialTrajectory(const Path& path, size_t nPerTrajectorySegment) {
        size_t waypointCount = path.Length();
        casadi::Slice all;
        casadi::DM X(3, (waypointCount - 1) * nPerTrajectorySegment + 1);
        for (size_t i = 0; i < waypointCount - 1; i++) {
            linspace(X, 2, path.GetWaypoint(i).heading, path.GetWaypoint(i + 1).heading, nPerTrajectorySegment);
            size_t guessPointsCount = path.GetWaypoint(i).initialGuessPoints.size();
            if (guessPointsCount == 0) {
                linspace(X, 0, path.GetWaypoint(i).x, path.GetWaypoint(i + 1).x, nPerTrajectorySegment);
                linspace(X, 1, path.GetWaypoint(i).y, path.GetWaypoint(i + 1).y, nPerTrajectorySegment);
                linspace(X, 2, path.GetWaypoint(i).heading, path.GetWaypoint(i + 1).heading, nPerTrajectorySegment);
            } else {
                size_t segmentsPerGuessSegment = nPerTrajectorySegment / guessPointsCount;
                linspace(X, 0, path.GetWaypoint(i).x, path.GetWaypoint(i).initialGuessPoints[0].x, segmentsPerGuessSegment);
                linspace(X, 1, path.GetWaypoint(i).y, path.GetWaypoint(i).initialGuessPoints[0].y, segmentsPerGuessSegment);
                linspace(X, 2, path.GetWaypoint(i).heading, path.GetWaypoint(i).initialGuessPoints[0].heading, segmentsPerGuessSegment);
                for (int j = 0; j < guessPointsCount - 1; j++) {
                    linspace(X, 0, path.GetWaypoint(i).initialGuessPoints[j].x, path.GetWaypoint(i).initialGuessPoints[j + 1].x, segmentsPerGuessSegment);
                    linspace(X, 1, path.GetWaypoint(i).initialGuessPoints[j].y, path.GetWaypoint(i).initialGuessPoints[j + 1].y, segmentsPerGuessSegment);
                    linspace(X, 2, path.GetWaypoint(i).initialGuessPoints[j].heading, path.GetWaypoint(i).initialGuessPoints[j + 1].heading, segmentsPerGuessSegment);
                }
                linspace(X, 0, path.GetWaypoint(i).initialGuessPoints[guessPointsCount - 1].x, path.GetWaypoint(i + 1).x, nPerTrajectorySegment - guessPointsCount * segmentsPerGuessSegment);
                linspace(X, 1, path.GetWaypoint(i).initialGuessPoints[guessPointsCount - 1].y, path.GetWaypoint(i + 1).y, nPerTrajectorySegment - guessPointsCount * segmentsPerGuessSegment);
                linspace(X, 1, path.GetWaypoint(i).initialGuessPoints[guessPointsCount - 1].heading, path.GetWaypoint(i + 1).heading, nPerTrajectorySegment - guessPointsCount * segmentsPerGuessSegment);
            }
        }
        X(0, (waypointCount - 1) * nPerTrajectorySegment) = path.GetWaypoint(waypointCount - 1).x;
        X(1, (waypointCount - 1) * nPerTrajectorySegment) = path.GetWaypoint(waypointCount - 1).y;
        X(2, (waypointCount - 1) * nPerTrajectorySegment) = path.GetWaypoint(waypointCount - 1).heading;

        return X;
    }

    void printHolonomicPath(const HolonomicPath& path) {
        std::cout << "[\n";
        for (const HolonomicWaypoint& waypoint : path.waypoints) {
            std::cout << "    {\n";
            std::cout << "        \"x\": " << waypoint.x << ",\n";
            std::cout << "        \"y\": " << waypoint.y << ",\n";
            std::cout << "        \"heading\": " << waypoint.heading << "\n";
            std::cout << "    },\n";
        }
        std::cout << "]" << std::endl;
    }

    void printHolonomicTrajectory(const HolonomicTrajectory& trajectory) {
        std::cout << "[\n";
        for (const HolonomicTrajectorySample& samp : trajectory.samples) {
            std::cout << "    {\n";
            std::cout << "        \"ts\": " << samp.ts << ",\n";
            std::cout << "        \"x\": " << samp.x << ",\n";
            std::cout << "        \"y\": " << samp.y << ",\n";
            std::cout << "        \"heading\": " << samp.heading << ",\n";
            std::cout << "        \"vx\": " << samp.vx << ",\n";
            std::cout << "        \"vy\": " << samp.vy << ",\n";
            std::cout << "        \"omega\": " << samp.omega << "\n";
            std::cout << "    },\n";
        }
        std::cout << "]" << std::endl;
    }
}