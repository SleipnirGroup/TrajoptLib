// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/constraint/AngularVelocityConstraint.hpp"
#include "trajopt/constraint/Constraint.hpp"
#include "trajopt/constraint/differential/DifferentialCentripetalAccelerationConstraint.hpp"
#include "trajopt/constraint/differential/DifferentialTangentialVelocityConstraint.hpp"
#include "trajopt/util/VariantCat.hpp"

namespace trajopt {

using DifferentialConstraint =
    variant_cat_t<Constraint, AngularVelocityConstraint,
                  DifferentialTangentialVelocityConstraint,
                  DifferentialCentripetalAccelerationConstraint>;

}  // namespace trajopt
