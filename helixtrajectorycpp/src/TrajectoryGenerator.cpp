#include "TrajectoryGenerator.h"

namespace helixtrajectory {
    TrajectoryGenerator::TrajectoryGenerator(const Drive& drive, const Path& path, const std::vector<Obstacle>& obstacles) :
            drive(drive), path(path), obstacles(obstacles),
            waypointCount(path.Length()), trajectorySegmentCount(waypointCount - 1), 
            nPerTrajectorySegment(100), nTotal(nPerTrajectorySegment * trajectorySegmentCount), opti(),
            trajectorySegmentTs(opti.variable(1, trajectorySegmentCount)),
            trajectorySegmentDts(trajectorySegmentTs / nPerTrajectorySegment) {
        casadi::MX totalT = 0;
        for (size_t i = 0; i < trajectorySegmentCount; i++) {
            totalT += trajectorySegmentTs(i);
            opti.subject_to(trajectorySegmentTs(i) >= 0);
            opti.set_initial(trajectorySegmentTs(i), 5);
        }
        std::cout << "Applied Time Constraints" << std::endl;
        opti.minimize(totalT);
        std::cout << "Set Optimization Objective" << std::endl;
    }
}