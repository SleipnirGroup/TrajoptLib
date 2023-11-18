// Copyright (c) TrajoptLib contributors

#include "trajopt/trajectory/HolonomicTrajectory.h"

#include <memory>

#include "trajopt/solution/HolonomicSolution.h"

namespace trajopt {

HolonomicTrajectory::HolonomicTrajectory(
    std::vector<HolonomicTrajectorySample> samples)
    : samples(std::move(samples)) {}

HolonomicTrajectory::HolonomicTrajectory(const HolonomicSolution& solution) {
  double ts = 0.0;
  for (size_t samp = 0; samp < solution.x.size(); samp++) {
    if (samp != 0) {
      ts += solution.dt[samp - 1];
    }
    samples.emplace_back(ts, solution.x[samp], solution.y[samp],
                         std::atan2(solution.thetasin[samp], solution.thetacos[samp]), solution.vx[samp],
                         solution.vy[samp], solution.omega[samp]);
  }
}
}  // namespace trajopt
