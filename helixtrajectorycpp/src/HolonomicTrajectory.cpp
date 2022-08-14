#include "HolonomicTrajectory.h"

#include <iostream>
#include <vector>

namespace helixtrajectory {

    HolonomicTrajectory::HolonomicTrajectory(const std::vector<HolonomicTrajectorySegment>& holonomicSegments)
        : holonomicSegments(holonomicSegments) {
    }

    HolonomicTrajectory::~HolonomicTrajectory() {
    }

    std::ostream& operator<<(std::ostream& stream, const HolonomicTrajectory& trajectory) {
        return stream << trajectory.holonomicSegments;
    }
}