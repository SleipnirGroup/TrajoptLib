#include <iostream>
#include <array>

#include "TrajectoryUtil.h"

namespace helixtrajectory {
    template<typename T>
    std::vector<double> linspace(T start_in, T end_in, int num_in) {
        std::vector<double> linspaced;
        linspaced.reserve(num_in);

        double start = static_cast<double>(start_in);
        double end = static_cast<double>(end_in);
        double num = static_cast<double>(num_in);

        if (num == 0) {
            return linspaced;
        }
        if (num == 1) {
            linspaced.push_back(start);
            return linspaced;
        }

        double delta = (end - start) / (num - 1);

        for (int i = 0; i < num - 1; ++i)
        {
            linspaced.push_back(start + delta * i);
        }
        linspaced.push_back(end);
        return linspaced;
    }

    casadi::DM generateInitialTrajectory(Path& path) {
        size_t waypointCount = path.waypoints.size();
        casadi::DM X(3, (waypointCount - 1) * N_PER_TRAJECTORY_SEGMENT + 1);
        for (int i = 0; i < waypointCount - 1; i++) {
            double startX = path.waypoints[i].x;
            double endX = path.waypoints[i + 1].x;
            double deltaX = (endX - startX) / N_PER_TRAJECTORY_SEGMENT;
            double startY = path.waypoints[i].y;
            double endY = path.waypoints[i + 1].y;
            double deltaY = (endY - startY) / N_PER_TRAJECTORY_SEGMENT;
            double startHeading = path.waypoints[i].heading;
            double endHeading = path.waypoints[i + 1].heading;
            double deltaHeading = (endHeading - startHeading) / N_PER_TRAJECTORY_SEGMENT;
            for (int j = 0; j < N_PER_TRAJECTORY_SEGMENT; j++) {
                X(0, i * N_PER_TRAJECTORY_SEGMENT + j) = startX + j * deltaX;
                X(1, i * N_PER_TRAJECTORY_SEGMENT + j) = startY + j * deltaY;
                X(2, i * N_PER_TRAJECTORY_SEGMENT + j) = startHeading + j * deltaHeading;
            }
        }
        X(0, (waypointCount - 1) * N_PER_TRAJECTORY_SEGMENT) = path.waypoints[waypointCount - 1].x;
        X(1, (waypointCount - 1) * N_PER_TRAJECTORY_SEGMENT) = path.waypoints[waypointCount - 1].y;
        X(2, (waypointCount - 1) * N_PER_TRAJECTORY_SEGMENT) = path.waypoints[waypointCount - 1].heading;

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