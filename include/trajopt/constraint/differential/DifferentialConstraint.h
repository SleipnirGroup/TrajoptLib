// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/constraint/AngularVelocityConstraint.h"
#include "trajopt/constraint/Constraint.h"
#include "trajopt/constraint/differential/DifferentialCentripetalAccelerationConstraint.h"
#include "trajopt/constraint/differential/DifferentialTangentialVelocityConstraint.h"
#include "trajopt/util/AppendVariant.h"

namespace trajopt {

using DifferentialConstraint =
    decltype(_append_variant(Constraint{}, AngularVelocityConstraint{},
                             DifferentialTangentialVelocityConstraint{},
                             DifferentialCentripetalAccelerationConstraint{}));

}  // namespace trajopt
