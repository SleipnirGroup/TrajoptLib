#include "HolonomicTrajectorySegment.h"

#include <iostream>

#include <casadi/casadi.hpp>

namespace helixtrajectory {

    HolonomicTrajectorySegment::HolonomicTrajectorySegment(double dt, const std::vector<HolonomicTrajectorySample>& holonomicSamples)
            : TrajectorySegment(dt), holonomicSamples(holonomicSamples) {
    }

    HolonomicTrajectorySegment::~HolonomicTrajectorySegment() {
    }

    std::ostream& operator<<(std::ostream& stream, const HolonomicTrajectorySegment& segment) {
        return stream << "{\"dt\": " << segment.dt
                << ", \"samples\": " << segment.holonomicSamples
                << "}";
    }
}