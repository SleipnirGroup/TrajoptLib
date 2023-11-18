// Copyright (c) TrajoptLib contributors

#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "DebugOptions.h"
#include "optimization/SwerveTrajoptUtil.h"
#include "optimization/TrajoptUtil.h"
#include "optimization/algorithms/SwerveDiscreteOptimal.h"
#include "trajopt/TrajectoryGenerationException.h"
#include "trajopt/drivetrain/SwerveDrivetrain.h"
#include "trajopt/path/Path.h"
#include "trajopt/solution/SwerveSolution.h"

namespace trajopt {

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
SwerveSolution SwerveDiscreteOptimal<Expr, Opti>::Generate() {
  try {
    opti.Solve();
    return ConstructSwerveSolution(opti, x, y, thetacos, thetasin, vx, vy, omega, ax, ay,
                                   alpha, Fx, Fy, dt, N);
  } catch (const std::exception& e) {
    throw TrajectoryGenerationException("Error optimizing trajectory: " +
                                        std::string(e.what()));
  }
}

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
SwerveDiscreteOptimal<Expr, Opti>::SwerveDiscreteOptimal(
    const SwervePath& path, const std::vector<size_t>& N,
    const Solution& initialGuess)
    : path(path), N(N) {
  size_t wptCnt = 1 + N.size();
  size_t sgmtCnt = N.size();
  size_t sampTot = GetIdx(N, wptCnt, 0);
  size_t moduleCnt = path.drivetrain.modules.size();

  x.reserve(sampTot);
  y.reserve(sampTot);
  thetacos.reserve(sampTot);
  thetasin.reserve(sampTot);
  vx.reserve(sampTot);
  vy.reserve(sampTot);
  omega.reserve(sampTot);
  ax.reserve(sampTot);
  ay.reserve(sampTot);
  alpha.reserve(sampTot);

  Fx.reserve(sampTot);
  Fy.reserve(sampTot);
  for (size_t sampIdx = 0; sampIdx < sampTot; ++sampIdx) {
    auto& _Fx = Fx.emplace_back();
    auto& _Fy = Fy.emplace_back();
    _Fx.reserve(moduleCnt);
    _Fy.reserve(moduleCnt);
  }

  dt.reserve(sgmtCnt);

  for (size_t idx = 0; idx < sampTot; ++idx) {
    x.emplace_back(opti.DecisionVariable());
    y.emplace_back(opti.DecisionVariable());
    thetacos.emplace_back(opti.DecisionVariable());
    thetasin.emplace_back(opti.DecisionVariable());
    vx.emplace_back(opti.DecisionVariable());
    vy.emplace_back(opti.DecisionVariable());
    omega.emplace_back(opti.DecisionVariable());
    ax.emplace_back(opti.DecisionVariable());
    ay.emplace_back(opti.DecisionVariable());
    alpha.emplace_back(opti.DecisionVariable());

    for (size_t moduleIdx = 0; moduleIdx < moduleCnt; ++moduleIdx) {
      Fx.at(idx).emplace_back(opti.DecisionVariable());
      Fy.at(idx).emplace_back(opti.DecisionVariable());
    }
  }

  for (size_t sgmtIdx = 0; sgmtIdx < sgmtCnt; ++sgmtIdx) {
    dt.emplace_back(opti.DecisionVariable());
  }

  ApplyDiscreteTimeObjective(opti, dt, N);
  ApplyKinematicsConstraints(opti, x, y, thetacos, thetasin, vx, vy, omega, ax, ay, alpha,
                             dt, N);

  for (size_t idx = 0; idx < sampTot; ++idx) {
    auto [Fx_net, Fy_net] = SolveNetForce(Fx.at(idx), Fy.at(idx));
    ApplyDynamicsConstraints(
        opti, ax.at(idx), ay.at(idx), alpha.at(idx), Fx_net, Fy_net,
        SolveNetTorque(thetacos.at(idx), thetasin.at(idx), Fx.at(idx), Fy.at(idx),
                       path.drivetrain.modules),
        path.drivetrain.mass, path.drivetrain.moi);

    ApplyPowerConstraints(opti, thetacos.at(idx), thetasin.at(idx), vx.at(idx), vy.at(idx),
                          omega.at(idx), Fx.at(idx), Fy.at(idx),
                          path.drivetrain);
  }

  for (size_t wptIdx = 0; wptIdx < wptCnt; ++wptIdx) {
    for (auto& constraint : path.waypoints.at(wptIdx).waypointConstraints) {
      size_t idx = GetIdx(N, wptIdx + 1, 0) - 1;  // first idx of next wpt - 1
      ApplyHolonomicConstraint(
          opti, x.at(idx), y.at(idx), thetacos.at(idx), thetasin.at(idx), vx.at(idx), vy.at(idx),
          omega.at(idx), ax.at(idx), ay.at(idx), alpha.at(idx), constraint);
    }
  }  // TODO: try changing the path struct so instead of having waypoint objects
     //       it's just two vectors of waypoint constraints and segment
     //       constraints, the waypoint one would be one larger by size
  for (size_t sgmtIdx = 0; sgmtIdx < sgmtCnt; ++sgmtIdx) {
    for (auto& constraint : path.waypoints.at(sgmtIdx + 1).segmentConstraints) {
      size_t startIdx = GetIdx(N, sgmtIdx + 1, 0);
      size_t endIdx = GetIdx(N, sgmtIdx + 2, 0);
      for (size_t idx = startIdx; idx < endIdx; ++idx) {
        ApplyHolonomicConstraint(
            opti, x.at(idx), y.at(idx), thetacos.at(idx), thetasin.at(idx), vx.at(idx), vy.at(idx),
            omega.at(idx), ax.at(idx), ay.at(idx), alpha.at(idx), constraint);
      }
    }
  }

  ApplyInitialGuess(opti, initialGuess, x, y, thetacos, thetasin);
}
}  // namespace trajopt
