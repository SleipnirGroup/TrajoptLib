#include "TrajectoryGenerator.h"

#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>

#include <casadi/casadi.hpp>

#include "Drive.h"
#include "HolonomicPath.h"
#include "HolonomicTrajectory.h"
#include "Obstacle.h"
#include "Path.h"
#include "TrajectoryUtil.h"

namespace helixtrajectory {
    TrajectoryGenerator::TrajectoryGenerator(const Drive& drive, const Path& path, const std::vector<Obstacle>& obstacles) :
            drive(drive), path(path), obstacles(obstacles),
            waypointCount(path.Length()), trajectorySegmentCount(waypointCount - 1), 
            nPerTrajectorySegment(100), nTotal(nPerTrajectorySegment * trajectorySegmentCount), opti(),
            trajectorySegmentTs(opti.variable(1, trajectorySegmentCount)),
            trajectorySegmentDts(trajectorySegmentTs / nPerTrajectorySegment),
            X(opti.variable(3, nTotal + 1)), x(X(0, ALL)), y(X(1, ALL)), theta(X(2, ALL)) {
        std::cout << "Number of Total Sample Segments " << nTotal << std::endl;

        casadi::MX totalT = 0;
        for (size_t i = 0; i < trajectorySegmentCount; i++) {
            totalT += trajectorySegmentTs(i);
            opti.subject_to(trajectorySegmentTs(i) >= 0);
            opti.set_initial(trajectorySegmentTs(i), 5);
        }
        std::cout << "Applied Time Constraints" << std::endl;
        opti.minimize(totalT);
        std::cout << "Set Optimization Objective" << std::endl;

        ApplyPathConstraints();
        std::cout << "Applied Path constraints" << std::endl;

        drive.ApplyObstacleConstraints(opti, x, y, theta, nTotal, obstacles);
        std::cout << "Applied Obstacle constraints" << std::endl;

        opti.set_initial(X, generateInitialTrajectory(path, nPerTrajectorySegment));
        std::cout << "Set Initial Trajectory" << std::endl;
    }

    void TrajectoryGenerator::ApplyPathConstraints() {
        for (int i = 0; i < waypointCount; i++) {
            int index = i * nPerTrajectorySegment;
            const Waypoint& waypoint = path.GetWaypoint(i);
            if (waypoint.xConstrained) {
                opti.subject_to(x(index) == waypoint.x);
            }
            if (waypoint.yConstrained) {
                opti.subject_to(y(index) == waypoint.y);
            }
            if (waypoint.headingConstrained) {
                // opti.subject_to(fmod(theta(index) - waypoint.heading, 2 * M_PI) == 0.0);
                opti.subject_to(theta(index) == waypoint.heading);
            }
        }
    }
}