// Copyright (c) TrajoptLib contributors

#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include <fmt/core.h>

#include "IncompatibleTrajectoryException.h"
#include "OptimalTrajectoryGenerator.h"
#include "constraint/Constraint.h"
#include "constraint/TranslationConstraint.h"
#include "constraint/holonomic/HolonomicConstraint.h"
#include "drivetrain/SwerveDrivetrain.h"
#include "obstacle/Obstacle.h"
#include "path/Path.h"
#include "path/InitialGuessPoint.h"
#include "path/SwervePathBuilder.h"
#include "set/ConeSet2d.h"
#include "solution/SwerveSolution.h"
#include "trajectory/HolonomicTrajectory.h"

int main() {
  using namespace trajopt;
  SwerveDrivetrain swerveDrivetrain{.mass = 45, .moi = 6,
                                    .modules={{+0.6, +0.6, 0.04, 70, 2},
                                              {+0.6, -0.6, 0.04, 70, 2},
                                              {-0.6, +0.6, 0.04, 70, 2},
                                              {-0.6, -0.6, 0.04, 70, 2}}};

  Obstacle bumpers{0, {{+0.5, +0.5}, {-0.5, +0.5}, {-0.5, -0.5}, {+0.5, -0.5}}};

  // One Meter Forward
  SwervePathBuilder path;
  path.SetDrivetrain(swerveDrivetrain);
  path.PoseWpt(0, 0.0, 0.0, 0.0);
  path.PoseWpt(1, 5.0, 0.0, 0.0);
  path.WptZeroVelocity(0);
  path.WptZeroVelocity(1);
  path.ControlIntervalCounts({4});

  // SOLVE
  try {
    SwerveSolution solution =
        OptimalTrajectoryGenerator::Generate(path);
    fmt::print("[{}]\n", fmt::join(path.CalculateInitialGuess().x, ","));
    fmt::print("{}\n", solution);
    // fmt::print("{}\n", HolonomicTrajectory(solution));
  } catch (const std::exception& e) {
    fmt::print("{}", e.what());
  }
}