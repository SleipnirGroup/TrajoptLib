// Copyright (c) TrajoptLib contributors

#pragma once

#include <variant>

#include "trajopt/SymbolExports.h"
#include "trajopt/constraint/AngularVelocityConstraint.h"
#include "trajopt/constraint/Constraint.h"
#include "trajopt/constraint/differential/DifferentialCentripetalAccelerationConstraint.h"
#include "trajopt/constraint/differential/DifferentialTangentialVelocityConstraint.h"
#include "trajopt/solution/SolutionChecking.h"
#include "trajopt/util/AppendVariant.h"

namespace trajopt {

using DifferentialConstraint =
    decltype(_append_variant(Constraint{}, AngularVelocityConstraint{},
                             DifferentialTangentialVelocityConstraint{},
                             DifferentialCentripetalAccelerationConstraint{}));
}  // namespace trajopt
