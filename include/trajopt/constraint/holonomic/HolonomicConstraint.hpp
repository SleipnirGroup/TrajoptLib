// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/constraint/AngularVelocityConstraint.hpp"
#include "trajopt/constraint/Constraint.hpp"
#include "trajopt/constraint/PointAtConstraint.hpp"
#include "trajopt/constraint/holonomic/HolonomicVelocityConstraint.hpp"
#include "trajopt/util/VariantCat.hpp"

namespace trajopt {

using HolonomicConstraint =
    variant_cat_t<Constraint, AngularVelocityConstraint,
                  HolonomicVelocityConstraint, PointAtConstraint>;

}  // namespace trajopt
