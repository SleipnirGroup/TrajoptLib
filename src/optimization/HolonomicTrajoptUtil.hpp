// Copyright (c) TrajoptLib contributors

#pragma once

#include "optimization/TrajoptUtil.hpp"
#include "trajopt/constraint/HeadingConstraint.hpp"
#include "trajopt/constraint/LinePointConstraint.hpp"
#include "trajopt/constraint/PointAtConstraint.hpp"
#include "trajopt/constraint/PointLineConstraint.hpp"
#include "trajopt/constraint/PointPointConstraint.hpp"
#include "trajopt/constraint/TranslationConstraint.hpp"

namespace trajopt {

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
void ApplyHolonomicConstraint(Opti& opti, const Expr& x, const Expr& y,
                              const Expr& theta, const Expr& vx, const Expr& vy,
                              const Expr& omega, const Expr& ax, const Expr& ay,
                              const Expr& alpha,
                              const HolonomicConstraint& constraint) {
  if (std::holds_alternative<HolonomicVelocityConstraint>(constraint)) {
    const auto& velocityHolonomicConstraint =
        std::get<HolonomicVelocityConstraint>(constraint);
    ApplySet2dConstraint(opti, vx, vy,
                         velocityHolonomicConstraint.velocityBound);
  } else if (std::holds_alternative<AngularVelocityConstraint>(constraint)) {
    const auto& angularVelocityConstraint =
        std::get<AngularVelocityConstraint>(constraint);
    ApplyIntervalSet1dConstraint(
        opti, omega, angularVelocityConstraint.angularVelocityBound);
  } else if (std::holds_alternative<TranslationConstraint>(constraint)) {
    ApplyConstraint(opti, x, y, theta,
                    std::get<TranslationConstraint>(constraint));
  } else if (std::holds_alternative<HeadingConstraint>(constraint)) {
    ApplyConstraint(opti, x, y, theta, std::get<HeadingConstraint>(constraint));
  } else if (std::holds_alternative<LinePointConstraint>(constraint)) {
    ApplyConstraint(opti, x, y, theta,
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
    auto ux = dx / hypot(dx, dy);  // NOLINT
    auto uy = dy / hypot(dx, dy);  // NOLINT
    auto hx = cos(theta);          // NOLINT
    auto hy = sin(theta);          // NOLINT
    auto dot = hx * ux + hy * uy;

    ApplyIntervalSet1dConstraint(
        opti, dot, IntervalSet1d(std::cos(headingTolerance), 1.0));
  } else if (std::holds_alternative<PointLineConstraint>(constraint)) {
    ApplyConstraint(opti, x, y, theta,
                    std::get<PointLineConstraint>(constraint));
  } else if (std::holds_alternative<PointPointConstraint>(constraint)) {
    ApplyConstraint(opti, x, y, theta,
                    std::get<PointPointConstraint>(constraint));
  }  // TODO: Investigate a way to condense the code above
}

}  // namespace trajopt
