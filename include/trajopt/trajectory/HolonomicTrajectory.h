// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include "trajopt/SymbolExports.h"
#include "trajopt/solution/HolonomicSolution.h"
#include "trajopt/trajectory/HolonomicTrajectorySample.h"

namespace trajopt {

/**
 * Holonomic trajectory.
 */
class TRAJOPT_DLLEXPORT HolonomicTrajectory {
 public:
  /// Trajectory samples.
  std::vector<HolonomicTrajectorySample> samples;

  /**
   * Construct a HolonomicTrajectory from samples.
   *
   * @param samples The samples.
   */
  explicit HolonomicTrajectory(std::vector<HolonomicTrajectorySample> samples);

  /**
   * Construct a HolonomicTrajectory from a solution.
   *
   * @param solution The solution.
   */
  explicit HolonomicTrajectory(const HolonomicSolution& solution);
};

}  // namespace trajopt
