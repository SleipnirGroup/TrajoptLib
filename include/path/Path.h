#pragma once

#include <cstddef>
#include <optional>
#include <type_traits>
#include <vector>
#include "constraint/differential/DifferentialConstraint.h"
#include "constraint/holonomic/HolonomicConstraint.h"
#include "drivetrain/DifferentialDrivetrain.h"
#include "drivetrain/SwerveDrivetrain.h"
#include "path/InitialGuessPoint.h"

namespace trajopt {

template<typename Constraint, typename Drivetrain>
  requires((std::is_same<Constraint, HolonomicConstraint>::value 
      || std::is_same<Constraint, DifferentialConstraint>::value)
      && (std::is_same<Drivetrain, SwerveDrivetrain>::value
      || std::is_same<Drivetrain, DifferentialDrivetrain>::value))
struct TRAJOPT_DLLEXPORT Waypoint {
  std::vector<Constraint> waypointConstraints;
  std::vector<Constraint> segmentConstraints;
  // Big abstraction leak - plz remove at some point in far far future
  size_t controlIntervalCount;
  std::vector<InitialGuessPoint> initialGuessPoints;
  std::optional<Drivetrain> segmentDrivetrain;
};

template<typename Constraint, typename Drivetrain>
  requires((std::is_same<Constraint, HolonomicConstraint>::value 
      || std::is_same<Constraint, DifferentialConstraint>::value)
      && (std::is_same<Drivetrain, SwerveDrivetrain>::value
      || std::is_same<Drivetrain, DifferentialDrivetrain>::value))
struct TRAJOPT_DLLEXPORT Path {
  std::vector<Waypoint<Constraint, Drivetrain>> waypoints;
  std::vector<Constraint> globalConstraints;
  Drivetrain drivetrain;
};

using SwerveWaypoint = Waypoint<HolonomicConstraint, SwerveDrivetrain>;
using SwervePath = Path<HolonomicConstraint, SwerveDrivetrain>;

using DifferentialWaypoint = Waypoint<DifferentialConstraint, DifferentialDrivetrain>;
using DifferentialPath = Path<DifferentialConstraint, DifferentialDrivetrain>;
}
