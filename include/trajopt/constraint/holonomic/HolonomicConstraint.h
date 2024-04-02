// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/constraint/AngularVelocityConstraint.h"
#include "trajopt/constraint/Constraint.h"
#include "trajopt/constraint/PointAtConstraint.h"
#include "trajopt/constraint/holonomic/HolonomicVelocityConstraint.h"
#include "trajopt/util/AppendVariant.h"

namespace trajopt {

using HolonomicConstraint = decltype(_append_variant(
    Constraint{}, AngularVelocityConstraint{}, HolonomicVelocityConstraint{}, PointAtConstraint{}));

}  // namespace trajopt
