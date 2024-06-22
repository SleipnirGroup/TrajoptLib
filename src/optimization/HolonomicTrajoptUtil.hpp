// Copyright (c) TrajoptLib contributors

#pragma once

#include <sleipnir/optimization/OptimizationProblem.hpp>

#include "optimization/TrajoptUtil.hpp"
#include "trajopt/constraint/HeadingConstraint.hpp"
#include "trajopt/constraint/LinePointConstraint.hpp"
#include "trajopt/constraint/PointAtConstraint.hpp"
#include "trajopt/constraint/PointLineConstraint.hpp"
#include "trajopt/constraint/PointPointConstraint.hpp"
#include "trajopt/constraint/TranslationConstraint.hpp"
#include "trajopt/constraint/holonomic/HolonomicConstraint.hpp"

namespace trajopt {

inline void ApplyHolonomicConstraint(
    sleipnir::OptimizationProblem& problem, const sleipnir::Variable& x,
    const sleipnir::Variable& y, const sleipnir::Variable& thetacos,
    const sleipnir::Variable& thetasin, const sleipnir::Variable& vx,
    const sleipnir::Variable& vy, const sleipnir::Variable& omega,
    [[maybe_unused]] const sleipnir::Variable& ax,
    [[maybe_unused]] const sleipnir::Variable& ay,
    [[maybe_unused]] const sleipnir::Variable& alpha,
    const HolonomicConstraint& constraint) {
  if (std::holds_alternative<HolonomicVelocityConstraint>(constraint)) {
    const auto& velocityHolonomicConstraint =
        std::get<HolonomicVelocityConstraint>(constraint);
    ApplySet2dConstraint(problem, vx, vy,
                         velocityHolonomicConstraint.velocityBound);
  } else if (std::holds_alternative<AngularVelocityConstraint>(constraint)) {
    const auto& angularVelocityConstraint =
        std::get<AngularVelocityConstraint>(constraint);
    ApplyIntervalSet1dConstraint(
        problem, omega, angularVelocityConstraint.angularVelocityBound);
  } else if (std::holds_alternative<TranslationConstraint>(constraint)) {
    ApplyConstraint(problem, x, y, thetacos, thetasin,
                    std::get<TranslationConstraint>(constraint));
  } else if (std::holds_alternative<HeadingConstraint>(constraint)) {
    ApplyConstraint(problem, x, y, thetacos, thetasin,
                    std::get<HeadingConstraint>(constraint));
  } else if (std::holds_alternative<LinePointConstraint>(constraint)) {
    ApplyConstraint(problem, x, y, thetacos, thetasin,
                    std::get<LinePointConstraint>(constraint));
  } else if (std::holds_alternative<PointAtConstraint>(constraint)) {
    auto pointAtConstraint = std::get<PointAtConstraint>(constraint);
    double fieldPointX = pointAtConstraint.fieldPointX;
    double fieldPointY = pointAtConstraint.fieldPointY;
    double headingTolerance = pointAtConstraint.headingTolerance;
    /**
     * dx,dy = desired heading
     * ux,uy = unit vector of desired heading
     * hx,hy = heading
     * dot = dot product of ux,uy and hx,hy
     *
     * constrain dot to cos(1.0), which is colinear
     * and cos(thetaTolerance)
     */
    auto dx = fieldPointX - x;
    auto dy = fieldPointY - y;
    auto dot = thetacos * dx + thetasin * dy;
    problem.SubjectTo(dot >=
                      std::cos(headingTolerance) * sleipnir::hypot(dx, dy));
  } else if (std::holds_alternative<PointLineConstraint>(constraint)) {
    ApplyConstraint(problem, x, y, thetacos, thetasin,
                    std::get<PointLineConstraint>(constraint));
  } else if (std::holds_alternative<PointPointConstraint>(constraint)) {
    ApplyConstraint(problem, x, y, thetacos, thetasin,
                    std::get<PointPointConstraint>(constraint));
  }  // TODO: Investigate a way to condense the code above
}

}  // namespace trajopt
