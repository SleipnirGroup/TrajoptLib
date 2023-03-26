// Copyright (c) TrajoptLib contributors

#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include <fmt/core.h>

#include "IncompatibleTrajectoryException.h"
#include "OptimalTrajectoryGenerator.h"
#include "TestUtil.h"
#include "constraint/Constraint.h"
#include "constraint/ObstacleConstraint.h"
#include "constraint/PoseConstraint.h"
#include "constraint/TranslationConstraint.h"
#include "constraint/differential/DifferentialTangentialVelocityConstraint.h"
#include "constraint/holonomic/HolonomicConstraint.h"
#include "drivetrain/DifferentialDrivetrain.h"
#include "drivetrain/SwerveDrivetrain.h"
#include "obstacle/Obstacle.h"
#include "path/Path.h"
#include "path/InitialGuessPoint.h"
#include "set/ConeSet2d.h"
#include "solution/SwerveSolution.h"
#include "trajectory/HolonomicTrajectory.h"

int main() {
  using namespace trajopt;
  DifferentialDrivetrain diffDrive{.mass = 45, .moi = 6, .trackwidth = 10, {0.04, 70, 2}, {0.04, 70, 2}};

//   fmt::print("{}\n", diffDrive);


  Obstacle bumpers{0, {{+0.5, +0.5}, {-0.5, +0.5}, {-0.5, -0.5}, {+0.5, -0.5}}};
  DifferentialPath path{
      {DifferentialWaypoint{{PoseConstraint{RectangularSet2d{0, 0}, 0},
                          AngularVelocityConstraint{0},
                          DifferentialTangentialVelocityConstraint{0}},
                          {},
                          0, {InitialGuessPoint(0, 0, 0.0)}},
       DifferentialWaypoint{{PoseConstraint{RectangularSet2d{4, 0}, 0},
                          DifferentialTangentialVelocityConstraint{0},
                          AngularVelocityConstraint{0}},
                          {},
                          50, {InitialGuessPoint(4, 0, 0.0)}}}, {}, bumpers};

  // SOLVE
  SwerveSolution solution =
      OptimalTrajectoryGenerator::Generate(diffDrive, path);
  fmt::print("{}\n", solution);
  fmt::print("{}\n", HolonomicTrajectory(solution));
}
