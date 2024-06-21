// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/constraint/AngularVelocityConstraint.h"
#include "trajopt/constraint/Constraint.h"
#include "trajopt/constraint/PointAtConstraint.h"
#include "trajopt/constraint/holonomic/HolonomicVelocityConstraint.h"
#include "trajopt/util/VariantCat.h"

namespace trajopt {

using HolonomicConstraint =
    variant_cat_t<Constraint, AngularVelocityConstraint,
                  HolonomicVelocityConstraint, PointAtConstraint>;

}  // namespace trajopt
