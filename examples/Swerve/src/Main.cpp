// Copyright (c) TrajoptLib contributors

#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include <trajopt/OptimalTrajectoryGenerator.hpp>
#include <trajopt/constraint/Constraint.hpp>
#include <trajopt/constraint/TranslationConstraint.hpp>
#include <trajopt/constraint/holonomic/HolonomicConstraint.hpp>
#include <trajopt/drivetrain/SwerveDrivetrain.hpp>
#include <trajopt/obstacle/Obstacle.hpp>
#include <trajopt/path/InitialGuessPoint.hpp>
#include <trajopt/path/Path.hpp>
#include <trajopt/path/SwervePathBuilder.hpp>
#include <trajopt/set/ConeSet2d.hpp>
#include <trajopt/solution/SwerveSolution.hpp>
#include <trajopt/trajectory/HolonomicTrajectory.hpp>

int main() {
  using namespace trajopt;

  SwerveDrivetrain swerveDrivetrain{.mass = 45,
                                    .moi = 6,
                                    .modules = {{+0.6, +0.6, 0.04, 70, 2},
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
  [[maybe_unused]] auto solution =
      OptimalTrajectoryGenerator::Generate(path, true);
}
