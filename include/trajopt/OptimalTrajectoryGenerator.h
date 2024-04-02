// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/SymbolExports.h"
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
  /**
   * @brief Prevent instantiation
   */
  OptimalTrajectoryGenerator() = delete;

  /**
   * @brief Initializes and solves an optimization problem for a swerve
   * drivetrain. This function may throw an exception if the optimizer is unable
   * to find the optizer
   *
   * @param path the path
   * @return the optimized swerve trajectory solution
   */
  static SwerveSolution Generate(const SwervePathBuilder& path, uint32_t handle);
};
}  // namespace trajopt
