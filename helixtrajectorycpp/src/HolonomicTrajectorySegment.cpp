#include "HolonomicTrajectorySegment.h"

#include <iostream>

#include <casadi/casadi.hpp>

namespace helixtrajectory {

    HolonomicTrajectorySegment::HolonomicTrajectorySegment(double intervalDuration, const std::vector<HolonomicTrajectorySample>& holonomicSamples)
            : TrajectorySegment(intervalDuration), holonomicSamples(holonomicSamples) {
    }

    HolonomicTrajectorySegment::~HolonomicTrajectorySegment() {
    }

    std::ostream& operator<<(std::ostream& stream, const HolonomicTrajectorySegment& segment) {
        return stream << "{\"interval_duration\": " << segment.intervalDuration
                << ", \"samples\": " << segment.holonomicSamples
                << "}";
    }
}