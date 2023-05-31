// Copyright (c) TrajoptLib contributors

#pragma once

#include <memory>
#include <vector>

#include "obstacle/Obstacle.h"
#include "optimization/TrajoptUtil.h"
#include "solution/HolonomicSolution.h"

namespace trajopt {

template <typename Expr, typename Opti> requires OptiSys<Expr, Opti>
void ApplyHolonomicConstraint(
    Opti& opti,
    const Expr& x,
    const Expr& y,
    const Expr& theta,
    const Expr& vx,
    const Expr& vy,
    const Expr& omega,
    const Expr& ax,
    const Expr& ay,
    const Expr& alpha,
    const HolonomicConstraint& constraint);
}  // namespace trajopt

#include "optimization/HolonomicTrajectoryOptimizationProblem.inc"
