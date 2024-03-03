// Copyright (c) TrajoptLib contributors

#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include <trajopt/OptimalTrajectoryGenerator.h>
#include <trajopt/constraint/Constraint.h>
#include <trajopt/constraint/TranslationConstraint.h>
#include <trajopt/constraint/holonomic/HolonomicConstraint.h>
#include <trajopt/drivetrain/SwerveDrivetrain.h>
#include <trajopt/obstacle/Obstacle.h>
#include <trajopt/path/InitialGuessPoint.h>
#include <trajopt/path/Path.h>
#include <trajopt/path/SwervePathBuilder.h>
#include <trajopt/set/ConeSet2d.h>
#include <trajopt/solution/SwerveSolution.h>
#include <trajopt/trajectory/HolonomicTrajectory.h>

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
