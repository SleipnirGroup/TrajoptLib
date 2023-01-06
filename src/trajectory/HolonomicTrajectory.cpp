// Copyright (c) TrajoptLib contributors

#include "trajectory/HolonomicTrajectory.h"

#include <memory>

namespace trajopt {

HolonomicTrajectory::HolonomicTrajectory(
    const std::vector<HolonomicTrajectorySample>& samples)
    : samples(samples) {}

HolonomicTrajectory::HolonomicTrajectory(
    std::vector<HolonomicTrajectorySample>&& samples)
    : samples(std::move(samples)) {}
}  // namespace trajopt
