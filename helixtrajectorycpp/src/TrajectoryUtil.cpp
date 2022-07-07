#include <iostream>
#include <array>

#include "TrajectoryUtil.h"

namespace helixtrajectory {

    void linspace(casadi::DM x, double start, double end, double n) {
        double delta = (end - start) / n;
        for (int i = 0; i < n; i++) {
            x(i) = start + i * delta;
        }
    }

    casadi::DM generateInitialTrajectory(const Path& path, size_t nPerTrajectorySegment) {
        size_t waypointCount = path.waypoints.size();
        casadi::Slice all;
        casadi::DM X(3, (waypointCount - 1) * nPerTrajectorySegment + 1);
        casadi::DM x = X(0, all);
        casadi::DM y = X(1, all);
        casadi::DM heading = X(2, all);
        for (int i = 0; i < waypointCount - 1; i++) {
            linspace(heading, path.waypoints[i].heading, path.waypoints[i + 1].heading, nPerTrajectorySegment);
            size_t guessPointsCount = path.waypoints[i].initialGuessPoints.size();
            if (guessPointsCount == 0) {
                linspace(x, path.waypoints[i].x, path.waypoints[i + 1].x, nPerTrajectorySegment);
                linspace(y, path.waypoints[i].y, path.waypoints[i + 1].y, nPerTrajectorySegment);
            } else {
                size_t segmentsPerGuessSegment = nPerTrajectorySegment / guessPointsCount;
                linspace(x, path.waypoints[i].x, path.waypoints[i].initialGuessPoints[0].x, segmentsPerGuessSegment);
                linspace(y, path.waypoints[i].y, path.waypoints[i].initialGuessPoints[0].y, segmentsPerGuessSegment);
                for (int j = 0; j < guessPointsCount - 1; j++) {
                    linspace(x, path.waypoints[i].initialGuessPoints[j].x, path.waypoints[i].initialGuessPoints[j + 1].x, segmentsPerGuessSegment);
                    linspace(y, path.waypoints[i].initialGuessPoints[j].y, path.waypoints[i].initialGuessPoints[j + 1].y, segmentsPerGuessSegment);
                }
                linspace(x, path.waypoints[i].initialGuessPoints[guessPointsCount - 1].x, path.waypoints[i + 1].x, nPerTrajectorySegment - guessPointsCount * segmentsPerGuessSegment);
                linspace(y, path.waypoints[i].initialGuessPoints[guessPointsCount - 1].y, path.waypoints[i + 1].y, nPerTrajectorySegment - guessPointsCount * segmentsPerGuessSegment);
            }
        }
        x((waypointCount - 1) * nPerTrajectorySegment) = path.waypoints[waypointCount - 1].x;
        y((waypointCount - 1) * nPerTrajectorySegment) = path.waypoints[waypointCount - 1].y;
        heading((waypointCount - 1) * nPerTrajectorySegment) = path.waypoints[waypointCount - 1].heading;

        return X;
    }

    void printPath(Path& path) {
        std::cout << "[\n";
        for (const Waypoint& waypoint : path.waypoints) {
            std::cout << "    {\n";
            std::cout << "        \"x\": " << waypoint.x << ",\n";
            std::cout << "        \"y\": " << waypoint.y << ",\n";
            std::cout << "        \"heading\": " << waypoint.heading << "\n";
            std::cout << "    },\n";
        }
        std::cout << "]" << std::endl;
    }

    void printTrajectory(Trajectory& trajectory) {
        std::cout << "[\n";
        for (const TrajectorySample& samp : trajectory.samples) {
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