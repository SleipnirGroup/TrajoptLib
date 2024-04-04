// Copyright (c) TrajoptLib contributors

#pragma once

#include <string>
#include <vector>

#include "optimization/HolonomicTrajoptUtil.h"
#include "optimization/OptiSys.h"
#include "optimization/TrajoptUtil.h"
#include "trajopt/drivetrain/SwerveDrivetrain.h"
#include "trajopt/expected"
#include "trajopt/path/Path.h"
#include "trajopt/solution/SwerveSolution.h"

namespace trajopt {

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
class SwerveDiscreteOptimal {
 public:
  /**
   * Generates an optimal trajectory.
   *
   * This function may take a long time to complete.
   *
   * @param diagnostics Enables diagnostic prints.
   * @return Returns a holonomic trajectory on success, or a string containing a
   *   failure reason.
   */
  expected<SwerveSolution, std::string> Generate(bool diagnostics = false);

 private:
  /**
   * The swerve drivetrain.
   */
  const SwervePath& path;

  /// State Variables
  std::vector<Expr> x;
  std::vector<Expr> y;
  std::vector<Expr> theta;
  std::vector<Expr> vx;
  std::vector<Expr> vy;
  std::vector<Expr> omega;
  std::vector<Expr> ax;
  std::vector<Expr> ay;
  std::vector<Expr> alpha;

  /// Input Variables
  std::vector<std::vector<Expr>> Fx;
  std::vector<std::vector<Expr>> Fy;

  /// Time Variables
  std::vector<Expr> dt;

  /// Discretization Constants
  const std::vector<size_t>& N;

  Opti opti;

 public:
  /**
   * Construct a new CasADi Swerve Trajectory Optimization Problem
   * with a swerve drivetrain and holonomic path.
   *
   * @param swerveDrivetrain the swerve drivetrain
   * @param holonomicPath the holonomic path
   */
  explicit SwerveDiscreteOptimal(const SwervePath& path,
                                 const std::vector<size_t>& N,
                                 const Solution& initialGuess,
                                 int64_t handle = 0);
};
}  // namespace trajopt

#include "optimization/algorithms/SwerveDiscreteOptimal.inc"
