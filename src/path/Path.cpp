// Copyright (c) TrajoptLib contributors

#include "path/Path.h"

#include <vector>

#include "constraint/Constraint.h"
#include "obstacle/Obstacle.h"
#include "path/Path.h"

namespace trajopt {

template struct Waypoint<HolonomicConstraint, SwerveDrivetrain>;
template struct Path<HolonomicConstraint, SwerveDrivetrain>;

template struct Waypoint<DifferentialConstraint, DifferentialDrivetrain>;
template struct Path<DifferentialConstraint, DifferentialDrivetrain>;
}  // namespace trajopt
