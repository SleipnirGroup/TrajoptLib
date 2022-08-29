#include "HolonomicTrajectory.h"

#include <iostream>
#include <vector>

#include <casadi/casadi.hpp>

#include "HolonomicTrajectorySegment.h"

namespace helixtrajectory {

    HolonomicTrajectory::HolonomicTrajectory(const std::vector<HolonomicTrajectorySegment>& holonomicSegments)
            : holonomicSegments(holonomicSegments) {
    }

    std::ostream& operator<<(std::ostream& stream, const HolonomicTrajectory& trajectory) {
        return stream << trajectory.holonomicSegments;
    }
}