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
#include "constraint/TranslationConstraint.h"
#include "constraint/holonomic/HolonomicConstraint.h"
#include "drivetrain/SwerveDrivetrain.h"
#include "obstacle/Obstacle.h"
#include "path/Path.h"
#include "path/InitialGuessPoint.h"
#include "set/ConeSet2d.h"
#include "solution/SwerveSolution.h"
#include "trajectory/HolonomicTrajectory.h"

int main() {
  // auto opti = casadi::Opti();

  // auto variable1 = opti.variable(4, 1);
  // auto variable2 = opti.variable(4, 1);

  using namespace trajopt;
  SwerveDrivetrain swerveDrivetrain{.mass = 45, .moi = 6,
                                    .modules={{+0.6, +0.6, 0.04, 70, 2},
                                              {+0.6, -0.6, 0.04, 70, 2},
                                              {-0.6, +0.6, 0.04, 70, 2},
                                              {-0.6, -0.6, 0.04, 70, 2}}};

  fmt::print("{}\n", swerveDrivetrain);

  Obstacle bumpers{0, {{+0.5, +0.5}, {-0.5, +0.5}, {-0.5, -0.5}, {+0.5, -0.5}}};

  // // CIRCULAR PATH
  // HolonomicPath holonomicPath(HolonomicPath({
  //     HolonomicWaypoint(
  //         {Constraint(PoseConstraint(RectangularSet2d{ 4,  0},  0.0))},
  //         {HolonomicConstraint(VelocityConstraint{RectangularSet2d(0, 0)}),
  //         HolonomicConstraint(AngularVelocityConstraint(0.0))},
  //         {},
  //         {},
  //         0,
  //         {InitialGuessPoint( 4,  0,  0.0)}),
  //     HolonomicWaypoint(
  //         {PoseConstraint(RectangularSet2d{ 0,  4},  1.57)},
  //         {},
  //         {},{},
  //         10,
  //         {InitialGuessPoint( 0,  4,  1.57)}),
  //     HolonomicWaypoint(
  //         {PoseConstraint(RectangularSet2d{-4,  0},  0.0)},
  //         {},
  //         {},
  //         {},
  //         10,
  //         {InitialGuessPoint(-4,  0,  0.0)}),
  //     HolonomicWaypoint(
  //         {PoseConstraint(RectangularSet2d{ 0, -4}, -1.57)},
  //         {},
  //         {},
  //         {},
  //         10,
  //         {InitialGuessPoint( 0, -4, -1.57)}),
  //     HolonomicWaypoint(
  //         {PoseConstraint(RectangularSet2d{ 4,  0},  0.0)},
  //         {VelocityConstraint{RectangularSet2d{0, 0}},
  //         AngularVelocityConstraint(0.0)},
  //         {},
  //         {},
  //         10,
  //         {InitialGuessPoint( 4,  0,  0.0)})},
  //     bumpers));

  // HARD OBSTACLE TEST:
  // const std::vector<InitialGuessPoint> guesses = {
  //     { 0.0,  0.0, 0.0},
  //     { 2.00,  1.50, 0.0},
  //     { 3.60,  0.0, -std::numbers::pi / 2.0},
  //     { 2.00, -1.50, -std::numbers::pi},
  //     { 0.0,  0.0, -std::numbers::pi}
  // };
  // Obstacle cone(0.6, {{2.0, 0.0}});
  // HolonomicPath holonomicPath({
  //     HolonomicWaypoint({PoseConstraint(RectangularSet2d{ 0,  0},  0.0)},
  //     {VelocityConstraint(RectangularSet2d{0, 0}),
  //     AngularVelocityConstraint(0.0)}, {                        }, {},   0,
  //     {InitialGuessPoint( 0,  0,   0.0)}),
  //     HolonomicWaypoint({PoseConstraint(RectangularSet2d{ 0,  0},
  //     -std::numbers::pi)}, {VelocityConstraint(RectangularSet2d{0, 0}),
  //     AngularVelocityConstraint(0.0)}, {ObstacleConstraint(cone)}, {}, 40,
  //     guesses)}, bumpers);

  // SIMPLE OBSTACLE TEST:
  // const std::vector<InitialGuessPoint> guesses = {
  //     { 0.0,  1.60, 0.0},
  //     { 4.00,  0.0, 0.0}
  // };
  // Obstacle cone(1.0, {{2.0, 0.0}});
  // HolonomicPath holonomicPath({
  //     HolonomicWaypoint({PoseConstraint(RectangularSet2d{ 0,  0},  0.0)},
  //     {VelocityConstraint(RectangularSet2d{0, 0}),
  //     AngularVelocityConstraint(0.0)}, {                        }, {},   0,
  //     {InitialGuessPoint( 0,  0,   0.0)}),
  //     HolonomicWaypoint({PoseConstraint(RectangularSet2d{ 4,  0},  0.0)},
  //     {VelocityConstraint(RectangularSet2d{0, 0}),
  //     AngularVelocityConstraint(0.0)}, {ObstacleConstraint(cone)}, {}, 36,
  //     guesses)}, bumpers);

  // SIMPLE MOTION PROFILE
  // auto con = Constraint(TranslationConstraint(RectangularSet2d(0, 0)));
  // auto con = VelocityConstraint(RectangularSet2d(0, 0));
  // fmt::print("{}\n\n\n", con);
  SwervePath path{
      {SwerveWaypoint{{TranslationConstraint{RectangularSet2d{0, 0}},
                          HeadingConstraint{0},
                          HolonomicVelocityConstraint{RectangularSet2d{0, 0},
                                                      CoordinateSystem::kField},
                          AngularVelocityConstraint{0}},
                          {},
                          0, {InitialGuessPoint(0, 0, 0.0)}, std::nullopt},
       SwerveWaypoint{{TranslationConstraint{RectangularSet2d{4, 0}},
                          HeadingConstraint{0},
                          HolonomicVelocityConstraint{RectangularSet2d{0, 0},
                                             CoordinateSystem::kField},
                          AngularVelocityConstraint{0}},
                          {},
                          50, {InitialGuessPoint(4, 0, 0.0)}, std::nullopt}}, {BumpersConstraint{bumpers}}, swerveDrivetrain};

  // SOLVE
  SwerveSolution solution =
      OptimalTrajectoryGenerator::Generate(path);
  fmt::print("{}\n", solution);
  fmt::print("{}\n", HolonomicTrajectory(solution));
}
