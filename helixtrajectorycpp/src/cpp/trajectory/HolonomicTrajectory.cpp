#include <trajectory/HolonomicTrajectory.h>

#include <memory>

// #include "IncompatibleTrajectoryException.h"
// #include "trajectory/HolonomicState.h"

namespace helixtrajectory {

HolonomicTrajectory::HolonomicTrajectory(const std::vector<HolonomicTrajectorySample>& samples)
      : samples(samples) {
}

HolonomicTrajectory::HolonomicTrajectory(std::vector<HolonomicTrajectorySample>&& samples)
      : samples(std::move(samples)) {
}
}