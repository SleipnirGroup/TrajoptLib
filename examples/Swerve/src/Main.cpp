// Copyright (c) TrajoptLib contributors

#include <trajopt/OptimalTrajectoryGenerator.hpp>

int main() {
  trajopt::SwerveDrivetrain swerveDrivetrain{
      .mass = 45,
      .moi = 6,
      .modules = {{{+0.6, +0.6}, 0.04, 70, 2},
                  {{+0.6, -0.6}, 0.04, 70, 2},
                  {{-0.6, +0.6}, 0.04, 70, 2},
                  {{-0.6, -0.6}, 0.04, 70, 2}}};

  trajopt::Obstacle bumpers{
      0, {{+0.5, +0.5}, {-0.5, +0.5}, {-0.5, -0.5}, {+0.5, -0.5}}};

  // One Meter Forward
  trajopt::SwervePathBuilder path;
  path.SetDrivetrain(swerveDrivetrain);
  path.PoseWpt(0, 0.0, 0.0, 0.0);
  path.PoseWpt(1, 5.0, 0.0, 0.0);
  path.WptZeroVelocity(0);
  path.WptZeroVelocity(1);
  path.ControlIntervalCounts({4});

  // SOLVE
  [[maybe_unused]]
  auto solution = trajopt::OptimalTrajectoryGenerator::Generate(path, true);
}
