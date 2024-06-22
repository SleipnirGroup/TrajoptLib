// Copyright (c) TrajoptLib contributors

#pragma once

#include <utility>
#include <variant>

#include <sleipnir/optimization/OptimizationProblem.hpp>

#include "trajopt/constraint/AngularVelocityEqualityConstraint.hpp"
#include "trajopt/constraint/AngularVelocityMaxMagnitudeConstraint.hpp"
#include "trajopt/constraint/LinePointConstraint.hpp"
#include "trajopt/constraint/LinearVelocityDirectionConstraint.hpp"
#include "trajopt/constraint/LinearVelocityMaxMagnitudeConstraint.hpp"
#include "trajopt/constraint/PointAtConstraint.hpp"
#include "trajopt/constraint/PointLineConstraint.hpp"
#include "trajopt/constraint/PointPointConstraint.hpp"
#include "trajopt/constraint/PoseEqualityConstraint.hpp"
#include "trajopt/constraint/TranslationEqualityConstraint.hpp"
#include "trajopt/geometry/Pose2.hpp"
#include "trajopt/geometry/Translation2.hpp"

namespace trajopt {

/**
 * ConstraintType concept.
 */
template <typename T>
concept ConstraintType = requires(T t) {
  {
    t.Apply(std::declval<sleipnir::OptimizationProblem>(),
            std::declval<Pose2v>(), std::declval<Translation2v>(),
            std::declval<sleipnir::Variable>())
  };  // NOLINT(readability/braces)
};

using Constraint =
    std::variant<AngularVelocityEqualityConstraint,
                 AngularVelocityMaxMagnitudeConstraint, LinePointConstraint,
                 LinearVelocityDirectionConstraint,
                 LinearVelocityMaxMagnitudeConstraint, PointAtConstraint,
                 PointLineConstraint, PointPointConstraint,
                 PoseEqualityConstraint, TranslationEqualityConstraint>;

}  // namespace trajopt
