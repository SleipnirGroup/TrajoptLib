// Copyright (c) TrajoptLib contributors

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
#include "solution/DifferentialSolution.h"
#include "solution/SwerveSolution.h"

namespace trajopt {

template <typename Constraint, typename Drivetrain>
requires((std::is_same<Constraint, HolonomicConstraint>::value &&
          std::is_same<Drivetrain, SwerveDrivetrain>::value) ||
         (std::is_same<Constraint, DifferentialConstraint>::value &&
          std::is_same<Drivetrain,
                       DifferentialDrivetrain>::value)) struct TRAJOPT_DLLEXPORT
    Waypoint {
  std::vector<Constraint> waypointConstraints;
  std::vector<Constraint> segmentConstraints;
  // std::optional<Drivetrain> segmentDrivetrain;
};

template <typename Constraint, typename Drivetrain, typename Solution>
requires(
    (std::is_same<Constraint, HolonomicConstraint>::value &&
     std::is_same<Drivetrain, SwerveDrivetrain>::value &&
     std::is_same<Solution, SwerveSolution>::value) ||
    (std::is_same<Constraint, DifferentialConstraint>::value &&
     std::is_same<Drivetrain, DifferentialDrivetrain>::value &&
     std::is_same<Solution,
                  DifferentialSolution>::value)) struct TRAJOPT_DLLEXPORT Path {
  std::vector<Waypoint<Constraint, Drivetrain>> waypoints;
  Drivetrain drivetrain;
};

using SwerveWaypoint = Waypoint<HolonomicConstraint, SwerveDrivetrain>;
using SwervePath = Path<HolonomicConstraint, SwerveDrivetrain, SwerveSolution>;

using DifferentialWaypoint =
    Waypoint<DifferentialConstraint, DifferentialDrivetrain>;
using DifferentialPath =
    Path<DifferentialConstraint, DifferentialDrivetrain, DifferentialSolution>;
}  // namespace trajopt
