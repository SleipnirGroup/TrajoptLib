// Copyright (c) TrajoptLib contributors

#pragma once

#include <algorithm>
#include <utility>
#include <vector>

#include "trajopt/SymbolExports.h"
#include "trajopt/solution/HolonomicSolution.h"
#include "trajopt/solution/SwerveSolution.h"
#include "trajopt/trajectory/HolonomicTrajectorySample.h"

namespace trajopt {

/**
 * Holonomic trajectory.
 */
class TRAJOPT_DLLEXPORT HolonomicTrajectory {
 public:
  /// Trajectory samples.
  std::vector<HolonomicTrajectorySample> samples;

  HolonomicTrajectory() = default;

  /**
   * Construct a HolonomicTrajectory from samples.
   *
   * @param samples The samples.
   */
  explicit HolonomicTrajectory(std::vector<HolonomicTrajectorySample> samples)
      : samples{std::move(samples)} {}

  /**
   * Construct a HolonomicTrajectory from a solution.
   *
   * @param solution The solution.
   */
  explicit HolonomicTrajectory(const HolonomicSolution& solution) {
    double ts = 0.0;
    for (size_t samp = 0; samp < solution.x.size(); ++samp) {
      if (samp != 0) {
        ts += solution.dt[samp - 1];
      }
      samples.emplace_back(ts, solution.x[samp], solution.y[samp],
                           solution.theta[samp], solution.vx[samp],
                           solution.vy[samp], solution.omega[samp]);
    }
  }

  /**
   * Construct a HolonomicTrajectory from a swerve solution.
   *
   * @param solution The swerve solution.
   */
  explicit HolonomicTrajectory(const SwerveSolution& solution) {
    double ts = 0.0;
    for (size_t samp = 0; samp < solution.x.size(); ++samp) {
      if (samp != 0) {
        ts += solution.dt[samp - 1];
      }
      samples.emplace_back(ts, solution.x[samp], solution.y[samp],
                           solution.theta[samp], solution.vx[samp],
                           solution.vy[samp], solution.omega[samp], solution.moduleFX[samp], solution.moduleFY[samp]);
    }
  }
};

}  // namespace trajopt
