// Copyright (c) TrajoptLib contributors

#pragma once

#include <cstddef>
#include <optional>
#include <type_traits>
#include <vector>

#include "SymbolExports.h"
#include "constraint/differential/DifferentialConstraint.h"
#include "constraint/holonomic/HolonomicConstraint.h"
#include "drivetrain/DifferentialDrivetrain.h"
#include "drivetrain/SwerveDrivetrain.h"
#include "path/InitialGuessPoint.h"
#include "solution/DifferentialSolution.h"
#include "solution/SwerveSolution.h"

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
