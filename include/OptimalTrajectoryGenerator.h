// Copyright (c) TrajoptLib contributors

#pragma once

#include "SymbolExports.h"
#include "path/Path.h"
#include "path/SwervePathBuilder.h"
#include "solution/SwerveSolution.h"

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
   * @param swerveDrivetrain the swerve drivetrain
   * @param holonomicPath the holonomic path
   * @return the optimized holonomic trajectory
   */
  static SwerveSolution Generate(const SwervePathBuilder& path);
};
}  // namespace trajopt
