// Copyright (c) TrajoptLib contributors

#pragma once

#include <variant>

#include <nlohmann/json.hpp>

#include "trajopt/SymbolExports.h"
#include "trajopt/constraint/AngularVelocityConstraint.h"
#include "trajopt/constraint/holonomic/HolonomicVelocityConstraint.h"
#include "trajopt/solution/SolutionChecking.h"
#include "trajopt/util/AppendVariant.h"
#include "trajopt/util/JsonFmtFormatter.h"

namespace trajopt {

using HolonomicConstraint = decltype(_append_variant(
    Constraint{}, AngularVelocityConstraint{}, HolonomicVelocityConstraint{}));

/**
 * Returns an error if the given state doesn't satisfy the constraint.
 *
 * @param x The x coordinate.
 * @param y The y coordinate.
 * @param heading The heading.
 * @param velocityX The velocity's x component.
 * @param velocityY The velocity's y component.
 * @param angularVelocity The angular velocity.
 * @param accelerationX The acceleration's x component.
 * @param accelerationY The acceleration's y component.
 * @param angularAcceleration The angular acceleration.
 * @param tolerances The tolerances considered to satisfy the constraint.
 */
std::optional<SolutionError> CheckState(
    const HolonomicConstraint& constraint, double x, double y, double heading,
    double velocityX, double velocityY, double angularVelocity,
    double accelerationX, double accelerationY, double angularAcceleration,
    const SolutionTolerances& tolerances) noexcept;

}  // namespace trajopt

// For the serialization functions of HolonomicConstraint, we want to use the
// serialization functions of the various constraint types
// (TranslationConstraint, HeadingConstraint, etc.), but the implicit
// conversion from each of those types to HolonomicConstraint (because of
// std::variant) requires using this style of serialization that avoids that
// issue.
//! @cond Doxygen_Suppress
namespace nlohmann {
  template <>
  struct adl_serializer<trajopt::HolonomicConstraint> {
    static void to_json(json& j, const trajopt::HolonomicConstraint& constraint);
    static void from_json(const json& j, trajopt::HolonomicConstraint& constraint);
  };
}
//! @endcond

_JSON_FMT_FORMATTER(trajopt::HolonomicConstraint)
