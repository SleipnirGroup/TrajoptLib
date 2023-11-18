// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include "DebugOptions.h"
#include "optimization/HolonomicTrajoptUtil.h"
#include "optimization/TrajoptUtil.h"
#include "trajopt/TrajectoryGenerationException.h"
#include "trajopt/constraint/Constraint.h"
#include "trajopt/constraint/HeadingConstraint.h"
#include "trajopt/constraint/LinePointConstraint.h"
#include "trajopt/constraint/PointLineConstraint.h"
#include "trajopt/constraint/PointPointConstraint.h"
#include "trajopt/constraint/TranslationConstraint.h"
#include "trajopt/obstacle/Obstacle.h"
#include "trajopt/path/Path.h"
#include "trajopt/solution/HolonomicSolution.h"

namespace trajopt {

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
void ApplyHolonomicConstraint(Opti& opti, const Expr& x, const Expr& y,
                              const Expr& thetacos, const Expr& thetasin, const Expr& vx, const Expr& vy,
                              const Expr& omega, const Expr& ax, const Expr& ay,
                              const Expr& alpha,
                              const HolonomicConstraint& holonomicConstraint) {
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
    ApplyConstraint(opti, x, y, thetacos, thetasin,
                    std::get<TranslationConstraint>(holonomicConstraint));
  } else if (std::holds_alternative<HeadingConstraint>(holonomicConstraint)) {
    ApplyConstraint(opti, x, y, thetacos, thetasin,
                    std::get<HeadingConstraint>(holonomicConstraint));
  } else if (std::holds_alternative<LinePointConstraint>(holonomicConstraint)) {
    ApplyConstraint(opti, x, y, thetacos, thetasin,
                    std::get<LinePointConstraint>(holonomicConstraint));
  } else if (std::holds_alternative<PointLineConstraint>(holonomicConstraint)) {
    ApplyConstraint(opti, x, y, thetacos, thetasin,
                    std::get<PointLineConstraint>(holonomicConstraint));
  } else if (std::holds_alternative<PointPointConstraint>(
                 holonomicConstraint)) {
    ApplyConstraint(opti, x, y, thetacos, thetasin,
                    std::get<PointPointConstraint>(holonomicConstraint));
  }  // TODO: Investigate a way to condense the code above
}
}  // namespace trajopt
