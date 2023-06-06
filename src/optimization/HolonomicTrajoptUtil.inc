// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include "DebugOptions.h"
#include "TrajectoryGenerationException.h"
#include "constraint/Constraint.h"
#include "constraint/HeadingConstraint.h"
#include "constraint/LinePointConstraint.h"
#include "constraint/PointLineConstraint.h"
#include "constraint/PointPointConstraint.h"
#include "constraint/TranslationConstraint.h"
#include "obstacle/Obstacle.h"
#include "optimization/HolonomicTrajoptUtil.h"
#include "optimization/TrajoptUtil.h"
#include "path/Path.h"
#include "solution/HolonomicSolution.h"

namespace trajopt {

template <typename Expr, typename Opti>
requires OptiSys<Expr, Opti> void ApplyHolonomicConstraint(
    Opti& opti, const Expr& x, const Expr& y, const Expr& theta, const Expr& vx,
    const Expr& vy, const Expr& omega, const Expr& ax, const Expr& ay,
    const Expr& alpha, const HolonomicConstraint& holonomicConstraint) {
  if (std::holds_alternative<HolonomicVelocityConstraint>(
          holonomicConstraint)) {
    const auto& velocityHolonomicConstraint =
        std::get<HolonomicVelocityConstraint>(holonomicConstraint);
    ApplySet2dConstraint(opti, vx, vy,
                         velocityHolonomicConstraint.velocityBound);
  } else if (std::holds_alternative<AngularVelocityConstraint>(
                 holonomicConstraint)) {
    const auto& angularVelocityConstraint =
        std::get<AngularVelocityConstraint>(holonomicConstraint);
    ApplyIntervalSet1dConstraint(
        opti, omega, angularVelocityConstraint.angularVelocityBound);
  } else if (std::holds_alternative<TranslationConstraint>(
                 holonomicConstraint)) {
    ApplyConstraint(opti, x, y, theta,
                    std::get<TranslationConstraint>(holonomicConstraint));
  } else if (std::holds_alternative<HeadingConstraint>(holonomicConstraint)) {
    ApplyConstraint(opti, x, y, theta,
                    std::get<HeadingConstraint>(holonomicConstraint));
  } else if (std::holds_alternative<LinePointConstraint>(holonomicConstraint)) {
    ApplyConstraint(opti, x, y, theta,
                    std::get<LinePointConstraint>(holonomicConstraint));
  } else if (std::holds_alternative<PointLineConstraint>(holonomicConstraint)) {
    ApplyConstraint(opti, x, y, theta,
                    std::get<PointLineConstraint>(holonomicConstraint));
  } else if (std::holds_alternative<PointPointConstraint>(
                 holonomicConstraint)) {
    ApplyConstraint(opti, x, y, theta,
                    std::get<PointPointConstraint>(holonomicConstraint));
  }  // TODO: Investigate a way to condense the code above
}
}  // namespace trajopt
