// Copyright (c) TrajoptLib contributors

#pragma once

#include <stdint.h>

#include <functional>
#include <vector>

#include "trajopt/constraint/differential/DifferentialConstraint.hpp"
#include "trajopt/constraint/holonomic/HolonomicConstraint.hpp"
#include "trajopt/drivetrain/DifferentialDrivetrain.hpp"
#include "trajopt/drivetrain/SwerveDrivetrain.hpp"
#include "trajopt/solution/SwerveSolution.hpp"
#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * A swerve path waypoint
 */
struct TRAJOPT_DLLEXPORT SwerveWaypoint {
  /// instantaneous constraints at the waypoint
  std::vector<HolonomicConstraint> waypointConstraints;
  /// continuous constraints along the segment
  std::vector<HolonomicConstraint> segmentConstraints;
};

/**
 * A differential path waypoint
 */
struct TRAJOPT_DLLEXPORT DifferentialWaypoint {
  /// instantaneous constraints at the waypoint
  std::vector<DifferentialConstraint> waypointConstraints;
  /// continuous constraints along the segment
  std::vector<DifferentialConstraint> segmentConstraints;
};

/**
 * Swerve path
 */
struct TRAJOPT_DLLEXPORT SwervePath {
  /// waypoints along the path
  std::vector<SwerveWaypoint> waypoints;
  /// drivetrain of the robot
  SwerveDrivetrain drivetrain;

  /// A vector of callbacks to be called with the intermediate SwerveSolution
  /// and a user-specified handle at every iteration of the solver
  std::vector<std::function<void(SwerveSolution&, int64_t)>> callbacks;
};

/**
 * Differential path
 */
struct TRAJOPT_DLLEXPORT DifferentialPath {
  /// waypoints along the path
  std::vector<DifferentialWaypoint> waypoints;
  /// drivetrain of the robot
  DifferentialDrivetrain drivetrain;
};

}  // namespace trajopt
