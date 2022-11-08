#pragma once

#include <variant>

#include "constraint/AngularVelocityConstraint.h"
#include "constraint/FieldRelativeVelocityHolonomicConstraint.h"

namespace helixtrajectory {

    using HolonomicConstraint = std::variant<FieldRelativeVelocityHolonomicConstraint,
                                             AngularVelocityConstraint>;
}