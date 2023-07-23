// Copyright (c) TrajoptLib contributors

#pragma once

#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "DebugOptions.h"
#include "optimization/SwerveTrajoptUtil.h"
#include "optimization/TrajoptUtil.h"
#include "trajopt/TrajectoryGenerationException.h"
#include "trajopt/drivetrain/SwerveDrivetrain.h"
#include "trajopt/path/Path.h"
#include "trajopt/solution/SwerveSolution.h"

namespace trajopt {

template <typename Expr>
  requires ExprSys<Expr>
std::pair<Expr, Expr> SolveNetForce(const std::vector<Expr>& Fx,
                                    const std::vector<Expr>& Fy) {
  Expr Fx_net = 0;
  Expr Fy_net = 0;

  for (auto& _Fx : Fx) {
    Fx_net += _Fx;
  }
  for (auto& _Fy : Fy) {
    Fy_net += _Fy;
  }

  return {Fx_net, Fy_net};
}

template <typename Expr>
  requires ExprSys<Expr>
Expr SolveNetTorque(const Expr& theta, const std::vector<Expr>& Fx,
                    const std::vector<Expr>& Fy,
                    const std::vector<SwerveModule>& swerveModules) {
  Expr tau_net = 0;

  for (size_t moduleIdx = 0; moduleIdx < swerveModules.size(); ++moduleIdx) {
    auto& swerveModule = swerveModules.at(moduleIdx);
    auto [x_m, y_m] =
        RotateConstantVector(swerveModule.x, swerveModule.y, theta);
    auto& Fx_m = Fx.at(moduleIdx);
    auto& Fy_m = Fy.at(moduleIdx);
    tau_net += x_m * Fy_m - y_m * Fx_m;
  }

  return tau_net;
}

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
void ApplyKinematicsConstraints(
    Opti& opti, const std::vector<Expr>& x, const std::vector<Expr>& y,
    const std::vector<Expr>& theta, const std::vector<Expr>& vx,
    const std::vector<Expr>& vy, const std::vector<Expr>& omega,
    const std::vector<Expr>& ax, const std::vector<Expr>& ay,
    const std::vector<Expr>& alpha, const std::vector<Expr>& dt,
    const std::vector<size_t> N) {
  size_t wptCnt = N.size() + 1;

  for (size_t wptIdx = 1; wptIdx < wptCnt; ++wptIdx) {
    size_t N_sgmt = N.at(wptIdx - 1);
    auto dt_sgmt = dt.at(wptIdx - 1);
    for (size_t sampIdx = 0; sampIdx < N_sgmt; ++sampIdx) {
      size_t idx = GetIdx(N, wptIdx, sampIdx);
      auto x_n = x.at(idx);
      auto x_n_1 = x.at(idx - 1);
      auto y_n = y.at(idx);
      auto y_n_1 = y.at(idx - 1);
      auto theta_n = theta.at(idx);
      auto theta_n_1 = theta.at(idx - 1);
      auto vx_n = vx.at(idx);
      auto vx_n_1 = vx.at(idx - 1);
      auto vy_n = vy.at(idx);
      auto vy_n_1 = vy.at(idx - 1);
      auto omega_n = omega.at(idx);
      auto omega_n_1 = omega.at(idx - 1);
      auto ax_n = ax.at(idx);
      auto ay_n = ay.at(idx);
      auto alpha_n = alpha.at(idx);
      opti.SubjectTo(x_n_1 + vx_n * dt_sgmt == x_n);
      opti.SubjectTo(y_n_1 + vy_n * dt_sgmt == y_n);
      opti.SubjectTo(cos(theta_n_1 + omega_n * dt_sgmt) ==  // NOLINT
                     cos(theta_n));                         // NOLINT
      opti.SubjectTo(sin(theta_n_1 + omega_n * dt_sgmt) ==  // NOLINT
                     sin(theta_n));                         // NOLINT
      opti.SubjectTo(vx_n_1 + ax_n * dt_sgmt == vx_n);
      opti.SubjectTo(vy_n_1 + ay_n * dt_sgmt == vy_n);
      opti.SubjectTo(omega_n_1 + alpha_n * dt_sgmt == omega_n);
    }
  }
}

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
void ApplyDynamicsConstraints(Opti& opti, const Expr& ax, const Expr& ay,
                              const Expr& alpha, const Expr& Fx_net,
                              const Expr& Fy_net, const Expr& tau_net,
                              double mass, double moi) {
  opti.SubjectTo(Fx_net == mass * ax);
  opti.SubjectTo(Fy_net == mass * ay);
  opti.SubjectTo(tau_net == moi * alpha);
}

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
void ApplyPowerConstraints(Opti& opti, const Expr& theta, const Expr& vx,
                           const Expr& vy, const Expr& omega,
                           const std::vector<Expr>& Fx,
                           const std::vector<Expr>& Fy,
                           const SwerveDrivetrain& swerveDrivetrain) {
  auto [vx_prime, vy_prime] = RotateVector(vx, vy, -theta);

  size_t moduleCount = swerveDrivetrain.modules.size();

  std::vector<Expr> vx_m;
  std::vector<Expr> vy_m;
  vx_m.reserve(moduleCount);
  vy_m.reserve(moduleCount);

  for (size_t moduleIdx = 0; moduleIdx < moduleCount; ++moduleIdx) {
    auto x_m = swerveDrivetrain.modules.at(moduleIdx).x;
    auto y_m = swerveDrivetrain.modules.at(moduleIdx).y;
    vx_m.emplace_back(vx_prime - y_m * omega);
    vy_m.emplace_back(vy_prime + x_m * omega);
  }

  for (size_t moduleIdx = 0; moduleIdx < moduleCount; ++moduleIdx) {
    auto& _module = swerveDrivetrain.modules.at(moduleIdx);
    double maxWheelVelocity =
        _module.wheelRadius * _module.wheelMaxAngularVelocity;
    double maxForce = _module.wheelMaxTorque / _module.wheelRadius;
    auto _vx_m = vx_m.at(moduleIdx);
    auto _vy_m = vy_m.at(moduleIdx);
    auto Fx_m = Fx.at(moduleIdx);
    auto Fy_m = Fy.at(moduleIdx);
    opti.SubjectTo(_vx_m * _vx_m + _vy_m * _vy_m <=
                   maxWheelVelocity * maxWheelVelocity);

    opti.SubjectTo(Fx_m * Fx_m + Fy_m * Fy_m <= maxForce * maxForce);
  }
}

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
SwerveSolution ConstructSwerveSolution(
    const Opti& opti, const std::vector<Expr>& x, const std::vector<Expr>& y,
    const std::vector<Expr>& theta, const std::vector<Expr>& vx,
    const std::vector<Expr>& vy, const std::vector<Expr>& omega,
    const std::vector<Expr>& ax, const std::vector<Expr>& ay,
    const std::vector<Expr>& alpha, const std::vector<std::vector<Expr>>& Fx,
    const std::vector<std::vector<Expr>>& Fy, const std::vector<Expr>& dt,
    const std::vector<size_t>& N) {
  std::vector<double> dtPerSamp;
  for (size_t sgmtIdx = 0; sgmtIdx < N.size(); ++sgmtIdx) {
    size_t N_sgmt = N.at(sgmtIdx);
    Expr dt_sgmt = dt.at(sgmtIdx);
    double dt_val = opti.SolutionValue(dt_sgmt);
    for (size_t i = 0; i < N_sgmt; ++i) {
      dtPerSamp.push_back(dt_val);
    }
  }
  return SwerveSolution{dtPerSamp,
                        RowSolutionValue(opti, x),
                        RowSolutionValue(opti, y),
                        RowSolutionValue(opti, theta),
                        RowSolutionValue(opti, vx),
                        RowSolutionValue(opti, vy),
                        RowSolutionValue(opti, omega),
                        RowSolutionValue(opti, ax),
                        RowSolutionValue(opti, ay),
                        RowSolutionValue(opti, alpha),
                        MatrixSolutionValue(opti, Fx),
                        MatrixSolutionValue(opti, Fy)};
}

}  // namespace trajopt
