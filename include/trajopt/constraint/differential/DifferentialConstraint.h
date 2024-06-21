// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/constraint/AngularVelocityConstraint.h"
#include "trajopt/constraint/Constraint.h"
#include "trajopt/constraint/differential/DifferentialCentripetalAccelerationConstraint.h"
#include "trajopt/constraint/differential/DifferentialTangentialVelocityConstraint.h"
#include "trajopt/util/VariantCat.h"

namespace trajopt {

using DifferentialConstraint =
    variant_cat_t<Constraint, AngularVelocityConstraint,
                  DifferentialTangentialVelocityConstraint,
                  DifferentialCentripetalAccelerationConstraint>;

}  // namespace trajopt
