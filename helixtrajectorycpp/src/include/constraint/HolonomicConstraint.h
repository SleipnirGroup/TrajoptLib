#pragma once

#include <variant>

#include "constraint/AngularVelocityConstraint.h"
#include "constraint/VelocityHolonomicConstraint.h"

namespace helixtrajectory {

using HolonomicConstraint = std::variant<VelocityHolonomicConstraint, AngularVelocityConstraint>;
}