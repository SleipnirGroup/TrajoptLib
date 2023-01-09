// Copyright (c) TrajoptLib contributors

#include "trajectory/HolonomicTrajectory.h"

#include <memory>

#include "solution/HolonomicSolution.h"

namespace trajopt {

HolonomicTrajectory::HolonomicTrajectory(
    const std::vector<HolonomicTrajectorySample>& samples)
    : samples(samples) {}

HolonomicTrajectory::HolonomicTrajectory(
    std::vector<HolonomicTrajectorySample>&& samples)
    : samples(std::move(samples)) {}

HolonomicTrajectory::HolonomicTrajectory(const HolonomicSolution& solution) {
  double ts = 0.0;
  for (size_t samp = 0; samp < solution.x.size(); samp++) {
    if (samp != 0) {
      ts += solution.dt[samp - 1];
    }
    samples.emplace_back(ts, solution.x[samp], solution.y[samp],
                         solution.theta[samp], solution.vx[samp],
                         solution.vy[samp], solution.omega[samp]);
  }
}
}  // namespace trajopt
