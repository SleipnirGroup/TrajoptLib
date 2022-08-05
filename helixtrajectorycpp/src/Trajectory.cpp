#include "HolonomicTrajectory.h"

#include <vector>

namespace helixtrajectory {

    HolonomicTrajectory::HolonomicTrajectory(const std::vector<HolonomicTrajectorySample>& samples)
        : samples(samples) {
    }
}