// Copyright (c) TrajoptLib contributors

#pragma once

#include <utility>
#include <vector>

#include <sleipnir/optimization/OptimizationProblem.hpp>

#include "optimization/TrajoptUtil.hpp"
#include "trajopt/drivetrain/SwerveDrivetrain.hpp"
#include "trajopt/solution/SwerveSolution.hpp"

namespace trajopt {

inline std::pair<sleipnir::Variable, sleipnir::Variable> SolveNetForce(
    const std::vector<sleipnir::Variable>& Fx,
    const std::vector<sleipnir::Variable>& Fy) {
  sleipnir::Variable Fx_net = 0;
  sleipnir::Variable Fy_net = 0;

  for (auto& _Fx : Fx) {
    Fx_net += _Fx;
  }
  for (auto& _Fy : Fy) {
    Fy_net += _Fy;
  }

  return {Fx_net, Fy_net};
}

inline sleipnir::Variable SolveNetTorque(
    const sleipnir::Variable& thetacos, const sleipnir::Variable& thetasin,
    const std::vector<sleipnir::Variable>& Fx,
    const std::vector<sleipnir::Variable>& Fy,
    const std::vector<SwerveModule>& swerveModules) {
  sleipnir::Variable tau_net = 0;

  for (size_t moduleIdx = 0; moduleIdx < swerveModules.size(); ++moduleIdx) {
    auto& swerveModule = swerveModules.at(moduleIdx);
    auto x_m = swerveModule.x * thetacos - swerveModule.y * thetasin;
    auto y_m = swerveModule.x * thetasin + swerveModule.y * thetacos;
    auto& Fx_m = Fx.at(moduleIdx);
    auto& Fy_m = Fy.at(moduleIdx);
    tau_net += x_m * Fy_m - y_m * Fx_m;
  }

  return tau_net;
}

inline void ApplyKinematicsConstraints(
    sleipnir::OptimizationProblem& problem,
    const std::vector<sleipnir::Variable>& x,
    const std::vector<sleipnir::Variable>& y,
    const std::vector<sleipnir::Variable>& thetacos,
    const std::vector<sleipnir::Variable>& thetasin,
    const std::vector<sleipnir::Variable>& vx,
    const std::vector<sleipnir::Variable>& vy,
    const std::vector<sleipnir::Variable>& omega,
    const std::vector<sleipnir::Variable>& ax,
    const std::vector<sleipnir::Variable>& ay,
    const std::vector<sleipnir::Variable>& alpha,
    const std::vector<sleipnir::Variable>& dt, const std::vector<size_t>& N) {
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
      auto theta_cos_n = thetacos.at(idx);
      auto theta_cos_n_1 = thetacos.at(idx - 1);
      auto theta_sin_n = thetasin.at(idx);
      auto theta_sin_n_1 = thetasin.at(idx - 1);
      auto vx_n = vx.at(idx);
      auto vx_n_1 = vx.at(idx - 1);
      auto vy_n = vy.at(idx);
      auto vy_n_1 = vy.at(idx - 1);
      auto omega_n = omega.at(idx);
      auto omega_n_1 = omega.at(idx - 1);
      auto ax_n = ax.at(idx);
      auto ay_n = ay.at(idx);
      auto alpha_n = alpha.at(idx);
      problem.SubjectTo(x_n_1 + vx_n * dt_sgmt == x_n);
      problem.SubjectTo(y_n_1 + vy_n * dt_sgmt == y_n);
      // Rotate theta_n by -theta_n_1 to get the difference
      auto theta_diff_cos =
          (theta_cos_n * theta_cos_n_1) - (theta_sin_n * -theta_sin_n_1);
      auto theta_diff_sin =
          (theta_cos_n * -theta_sin_n_1) + (theta_sin_n * theta_cos_n_1);
      // Constrain angle equality on manifold: theta_diff = omega_n * dt_sgmt.
      //
      // Let a = <cos(theta_diff), sin(theta_diff)>.  NOLINT
      // Let b = <cos(omega_n * dt_sgmt), sin(omega_n * dt_sgmt)>.  NOLINT
      //
      // If the angles are equal, the angle between the unit vectors should be
      // zero.
      //
      //   a x b = ||a|| ||b|| sin(angleBetween)  NOLINT
      //         = 1 * 1 * 0
      //         = 0
      //
      //   a x b = 0
      //   a.x * b.y - b.x * a.y = 0
      //   a.x * b.y = b.x * a.y
      //   cos(theta_diff) * sin(omega_n * dt_sgmt) =
      //     sin(theta_diff) * cos(omega_n * dt_sgmt)
      // NOTE: angleBetween = pi rad would be another solution
      problem.SubjectTo(theta_diff_cos * sleipnir::sin(omega_n * dt_sgmt) -
                            theta_diff_sin * sleipnir::cos(omega_n * dt_sgmt) ==
                        0);
      problem.SubjectTo(
          theta_cos_n_1 * theta_cos_n_1 + theta_sin_n_1 * theta_sin_n_1 == 1);
      problem.SubjectTo(vx_n_1 + ax_n * dt_sgmt == vx_n);
      problem.SubjectTo(vy_n_1 + ay_n * dt_sgmt == vy_n);
      problem.SubjectTo(omega_n_1 + alpha_n * dt_sgmt == omega_n);
    }
    size_t lastIdx = GetIdx(N, wptIdx, N_sgmt - 1);
    problem.SubjectTo(thetacos.at(lastIdx) * thetacos.at(lastIdx) +
                          thetasin.at(lastIdx) * thetasin.at(lastIdx) ==
                      1);
  }
}

/**
 * Applies the drivetrain-specific constraints to the optimizer. These
 * constraints prevent motors from spinning too fast or with too much power.
 * For swerve, this applies constraints that connect the speed and direction
 * of each swerve module wheel to the overall kinematics and dynamics of the
 * system. There are two parts: the velocity of each wheel is connected to the
 * velocity of the robot, and the force generated by each wheel is connected
 * to the acceleration of the robot. For both of these, limits are placed on
 * the speed and torque of each wheel. This allows the optimizer to generate
 * an efficient, smooth path that the robot can follow.
 *
 * @param problem the current optimization problem to which to apply constraints
 * @param theta (controlIntervalTotal + 1) x 1 column vector of the robot's
 * heading for each sample point
 * @param vx (controlIntervalTotal + 1) x 1 column vector of the x-coordinate
 * of the robot's velocity for each sample point
 * @param vy (controlIntervalTotal + 1) x 1 column vector of the y-coordinate
 * of the robot's velocity for each sample point
 * @param omega (controlIntervalTotal + 1) x 1 column vector of the robot's
 * angular velocity for each sample point
 * @param ax controlIntervalTotal x 1 column vector of the x-coordinate of the
 * robot's acceleration for each sample point
 * @param ay controlIntervalTotal x 1 column vector of the y-coordinate of the
 * robot's acceleration for each sample point
 * @param alpha controlIntervalTotal x 1 column vector of the robot's angular
 * velocity for each sample point
 * @param swerveDrivetrain the swerve drivetrain
 */
inline void ApplyDynamicsConstraints(
    sleipnir::OptimizationProblem& problem, const sleipnir::Variable& ax,
    const sleipnir::Variable& ay, const sleipnir::Variable& alpha,
    const sleipnir::Variable& Fx_net, const sleipnir::Variable& Fy_net,
    const sleipnir::Variable& tau_net, double mass, double moi) {
  problem.SubjectTo(Fx_net == mass * ax);
  problem.SubjectTo(Fy_net == mass * ay);
  problem.SubjectTo(tau_net == moi * alpha);
}

inline void ApplyPowerConstraints(sleipnir::OptimizationProblem& problem,
                                  const sleipnir::Variable& thetacos,
                                  const sleipnir::Variable& thetasin,
                                  const sleipnir::Variable& vx,
                                  const sleipnir::Variable& vy,
                                  const sleipnir::Variable& omega,
                                  const std::vector<sleipnir::Variable>& Fx,
                                  const std::vector<sleipnir::Variable>& Fy,
                                  const SwerveDrivetrain& swerveDrivetrain) {
  auto vx_prime = vx * thetacos - vy * -thetasin;
  auto vy_prime = vx * -thetasin + vy * thetacos;

  size_t moduleCount = swerveDrivetrain.modules.size();

  std::vector<sleipnir::Variable> vx_m;
  std::vector<sleipnir::Variable> vy_m;
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
    problem.SubjectTo(_vx_m * _vx_m + _vy_m * _vy_m <=
                      maxWheelVelocity * maxWheelVelocity);

    problem.SubjectTo(Fx_m * Fx_m + Fy_m * Fy_m <= maxForce * maxForce);
  }
}

inline SwerveSolution ConstructSwerveSolution(
    std::vector<sleipnir::Variable>& x, std::vector<sleipnir::Variable>& y,
    std::vector<sleipnir::Variable>& thetacos,
    std::vector<sleipnir::Variable>& thetasin,
    std::vector<sleipnir::Variable>& vx, std::vector<sleipnir::Variable>& vy,
    std::vector<sleipnir::Variable>& omega, std::vector<sleipnir::Variable>& ax,
    std::vector<sleipnir::Variable>& ay, std::vector<sleipnir::Variable>& alpha,
    std::vector<std::vector<sleipnir::Variable>>& Fx,
    std::vector<std::vector<sleipnir::Variable>>& Fy,
    std::vector<sleipnir::Variable>& dt, const std::vector<size_t>& N) {
  std::vector<double> dtPerSamp;
  for (size_t sgmtIdx = 0; sgmtIdx < N.size(); ++sgmtIdx) {
    size_t N_sgmt = N.at(sgmtIdx);
    sleipnir::Variable dt_sgmt = dt.at(sgmtIdx);
    double dt_val = dt_sgmt.Value();
    for (size_t i = 0; i < N_sgmt; ++i) {
      dtPerSamp.push_back(dt_val);
    }
  }
  return SwerveSolution{
      {{dtPerSamp, RowSolutionValue(x), RowSolutionValue(y),
        RowSolutionValue(thetacos), RowSolutionValue(thetasin)},
       RowSolutionValue(vx),
       RowSolutionValue(vy),
       RowSolutionValue(omega),
       RowSolutionValue(ax),
       RowSolutionValue(ay),
       RowSolutionValue(alpha)},
      MatrixSolutionValue(Fx),
      MatrixSolutionValue(Fy)};
}

}  // namespace trajopt
