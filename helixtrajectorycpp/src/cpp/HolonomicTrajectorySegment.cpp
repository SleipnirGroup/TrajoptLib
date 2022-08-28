#include "HolonomicTrajectorySegment.h"

#include <iostream>

#include <casadi/casadi.hpp>

namespace helixtrajectory {

    HolonomicTrajectorySegment::HolonomicTrajectorySegment(const std::vector<HolonomicTrajectorySample>& holonomicSamples)
            : TrajectorySegment(intervalDuration), holonomicSamples(holonomicSamples) {
    }

    HolonomicTrajectorySegment::~HolonomicTrajectorySegment() {
    }

    std::ostream& operator<<(std::ostream& stream, const HolonomicTrajectorySegment& segment) {
        return stream << segment.holonomicSamples;
    }
}