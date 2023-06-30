// Copyright (c) TrajoptLib contributors

#pragma once

#include <cstddef>
#include <optional>
#include <type_traits>
#include <vector>

#include "trajopt/SymbolExports.h"
#include "trajopt/constraint/differential/DifferentialConstraint.h"
#include "trajopt/constraint/holonomic/HolonomicConstraint.h"
#include "trajopt/drivetrain/DifferentialDrivetrain.h"
#include "trajopt/drivetrain/SwerveDrivetrain.h"
#include "trajopt/path/InitialGuessPoint.h"
#include "trajopt/solution/DifferentialSolution.h"
#include "trajopt/solution/SwerveSolution.h"

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
