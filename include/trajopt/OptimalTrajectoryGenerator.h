// Copyright (c) TrajoptLib contributors

#pragma once

#include <string>

#include "trajopt/SymbolExports.h"
#include "trajopt/expected"
#include "trajopt/path/Path.h"
#include "trajopt/path/SwervePathBuilder.h"
#include "trajopt/solution/SwerveSolution.h"

namespace trajopt {

/**
 * @brief This trajectory generator class contains functions to generate
 * time-optimal trajectories for several drivetrain types.
 */
class TRAJOPT_DLLEXPORT OptimalTrajectoryGenerator {
 public:
  OptimalTrajectoryGenerator() = delete;

  /**
   * Initializes and solves an optimization problem for a swerve drivetrain.
   *
   * @param path the path
   * @param diagnostics Enables diagnostic prints.
   * @return The optimized swerve trajectory solution on success, or a string
   *   containing the failure reason.
   */
  static expected<SwerveSolution, std::string> Generate(
      const SwervePathBuilder& path, bool diagnostics = false);
};

}  // namespace trajopt
