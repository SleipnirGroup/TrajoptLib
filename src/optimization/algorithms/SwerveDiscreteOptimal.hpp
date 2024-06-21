// Copyright (c) TrajoptLib contributors

#pragma once

#include <algorithm>
#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include <sleipnir/optimization/OptimizationProblem.hpp>

#include "optimization/Cancellation.hpp"
#include "optimization/HolonomicTrajoptUtil.hpp"
#include "optimization/SwerveTrajoptUtil.hpp"
#include "optimization/TrajoptUtil.hpp"
#include "trajopt/drivetrain/SwerveDrivetrain.hpp"
#include "trajopt/expected"
#include "trajopt/path/Path.hpp"
#include "trajopt/solution/SwerveSolution.hpp"

namespace trajopt {

class SwerveDiscreteOptimal {
 public:
  /**
   * Construct a new swerve trajectory optimization problem.
   */
  SwerveDiscreteOptimal(const SwervePath& path, const std::vector<size_t>& N,
                        const Solution& initialGuess, int64_t handle = 0)
      : path(path), N(N) {
    callbacks.emplace_back([this, handle = handle] {
      constexpr int fps = 60;
      constexpr std::chrono::duration<double> timePerFrame{1.0 / fps};

      // FPS limit on sending updates
      static auto lastFrameTime = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      if (now - lastFrameTime < timePerFrame) {
        return;
      }

      lastFrameTime = now;

      auto soln =
          ConstructSwerveSolution(x, y, thetacos, thetasin, vx, vy, omega, ax,
                                  ay, alpha, Fx, Fy, dt, this->N);
      for (auto& callback : this->path.callbacks) {
        callback(soln, handle);
      }
    });
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
      x.emplace_back(problem.DecisionVariable());
      y.emplace_back(problem.DecisionVariable());
      thetacos.emplace_back(problem.DecisionVariable());
      thetasin.emplace_back(problem.DecisionVariable());
      vx.emplace_back(problem.DecisionVariable());
      vy.emplace_back(problem.DecisionVariable());
      omega.emplace_back(problem.DecisionVariable());
      ax.emplace_back(problem.DecisionVariable());
      ay.emplace_back(problem.DecisionVariable());
      alpha.emplace_back(problem.DecisionVariable());

      for (size_t moduleIdx = 0; moduleIdx < moduleCnt; ++moduleIdx) {
        Fx.at(idx).emplace_back(problem.DecisionVariable());
        Fy.at(idx).emplace_back(problem.DecisionVariable());
      }
    }

    double minWidth = INFINITY;
    for (size_t i = 1; i < path.drivetrain.modules.size(); i++) {
      if (std::abs(path.drivetrain.modules.at(i - 1).translation.X() -
                   path.drivetrain.modules.at(i).translation.X()) != 0) {
        minWidth = std::min(
            minWidth,
            std::abs(path.drivetrain.modules.at(i - 1).translation.X() -
                     path.drivetrain.modules.at(i).translation.X()));
      }
      if (std::abs(path.drivetrain.modules.at(i - 1).translation.Y() -
                   path.drivetrain.modules.at(i).translation.Y()) != 0) {
        minWidth = std::min(
            minWidth,
            std::abs(path.drivetrain.modules.at(i - 1).translation.Y() -
                     path.drivetrain.modules.at(i).translation.Y()));
      }
    }

    for (size_t sgmtIdx = 0; sgmtIdx < sgmtCnt; ++sgmtIdx) {
      dt.emplace_back(problem.DecisionVariable());
      for (auto module : path.drivetrain.modules) {
        problem.SubjectTo(dt.at(sgmtIdx) * module.wheelRadius *
                              module.wheelMaxAngularVelocity <=
                          minWidth);
      }
    }

    ApplyDiscreteTimeObjective(problem, dt, N);
    ApplyKinematicsConstraints(problem, x, y, thetacos, thetasin, vx, vy, omega,
                               ax, ay, alpha, dt, N);

    for (size_t idx = 0; idx < sampTot; ++idx) {
      auto [Fx_net, Fy_net] = SolveNetForce(Fx.at(idx), Fy.at(idx));
      ApplyDynamicsConstraints(
          problem, ax.at(idx), ay.at(idx), alpha.at(idx), Fx_net, Fy_net,
          SolveNetTorque({thetacos.at(idx), thetasin.at(idx)}, Fx.at(idx),
                         Fy.at(idx), path.drivetrain.modules),
          path.drivetrain.mass, path.drivetrain.moi);

      ApplyPowerConstraints(problem,
                            Rotation2v{thetacos.at(idx), thetasin.at(idx)},
                            {vx.at(idx), vy.at(idx)}, omega.at(idx), Fx.at(idx),
                            Fy.at(idx), path.drivetrain);
    }

    for (size_t wptIdx = 0; wptIdx < wptCnt; ++wptIdx) {
      for (auto& constraint : path.waypoints.at(wptIdx).waypointConstraints) {
        size_t idx = GetIdx(N, wptIdx + 1, 0) - 1;  // first idx of next wpt - 1
        ApplyHolonomicConstraint(
            problem,
            {x.at(idx), y.at(idx), {thetacos.at(idx), thetasin.at(idx)}},
            {vx.at(idx), vy.at(idx)}, omega.at(idx), {ax.at(idx), ay.at(idx)},
            alpha.at(idx), constraint);
      }
    }
    // TODO: try changing the path struct so instead of having waypoint objects
    //       it's just two vectors of waypoint constraints and segment
    //       constraints, the waypoint one would be one larger by size
    for (size_t sgmtIdx = 0; sgmtIdx < sgmtCnt; ++sgmtIdx) {
      for (auto& constraint :
           path.waypoints.at(sgmtIdx + 1).segmentConstraints) {
        size_t startIdx = GetIdx(N, sgmtIdx + 1, 0);
        size_t endIdx = GetIdx(N, sgmtIdx + 2, 0);
        for (size_t idx = startIdx; idx < endIdx; ++idx) {
          ApplyHolonomicConstraint(
              problem,
              {x.at(idx), y.at(idx), {thetacos.at(idx), thetasin.at(idx)}},
              {vx.at(idx), vy.at(idx)}, omega.at(idx), {ax.at(idx), ay.at(idx)},
              alpha.at(idx), constraint);
        }
      }
    }

    ApplyInitialGuess(initialGuess, x, y, thetacos, thetasin, vx, vy, omega, ax,
                      ay, alpha);
  }

  /**
   * Generates an optimal trajectory.
   *
   * This function may take a long time to complete.
   *
   * @param diagnostics Enables diagnostic prints.
   * @return Returns a holonomic trajectory on success, or a string containing a
   *   failure reason.
   */
  expected<SwerveSolution, std::string> Generate(bool diagnostics = false) {
    GetCancellationFlag() = 0;
    problem.Callback([=, this](const sleipnir::SolverIterationInfo&) -> bool {
      for (auto& callback : callbacks) {
        callback();
      }
      return trajopt::GetCancellationFlag();
    });

    // tolerance of 1e-4 is 0.1 mm
    auto status =
        problem.Solve({.tolerance = 1e-4, .diagnostics = diagnostics});

    if (static_cast<int>(status.exitCondition) < 0 ||
        status.exitCondition ==
            sleipnir::SolverExitCondition::kCallbackRequestedStop) {
      return unexpected{std::string{sleipnir::ToMessage(status.exitCondition)}};
    } else {
      return ConstructSwerveSolution(x, y, thetacos, thetasin, vx, vy, omega,
                                     ax, ay, alpha, Fx, Fy, dt, N);
    }
  }

 private:
  /**
   * The swerve drivetrain.
   */
  const SwervePath& path;

  /// State Variables
  std::vector<sleipnir::Variable> x;
  std::vector<sleipnir::Variable> y;
  std::vector<sleipnir::Variable> thetacos;
  std::vector<sleipnir::Variable> thetasin;
  std::vector<sleipnir::Variable> vx;
  std::vector<sleipnir::Variable> vy;
  std::vector<sleipnir::Variable> omega;
  std::vector<sleipnir::Variable> ax;
  std::vector<sleipnir::Variable> ay;
  std::vector<sleipnir::Variable> alpha;

  /// Input Variables
  std::vector<std::vector<sleipnir::Variable>> Fx;
  std::vector<std::vector<sleipnir::Variable>> Fy;

  /// Time Variables
  std::vector<sleipnir::Variable> dt;

  /// Discretization Constants
  const std::vector<size_t>& N;

  sleipnir::OptimizationProblem problem;
  std::vector<std::function<void()>> callbacks;
};

}  // namespace trajopt
