// Copyright (c) TrajoptLib contributors

#pragma once

#include <stdint.h>

#include <string>

#include "trajopt/expected"
#include "trajopt/path/SwervePathBuilder.hpp"
#include "trajopt/solution/SwerveSolution.hpp"
#include "trajopt/util/SymbolExports.hpp"

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
   * @param handle an identifier for state callbacks
   * @return The optimized swerve trajectory solution on success, or a string
   *   containing the failure reason.
   */
  static expected<SwerveSolution, std::string> Generate(
      const SwervePathBuilder& path, bool diagnostics = false,
      int64_t handle = 0);
};

}  // namespace trajopt
