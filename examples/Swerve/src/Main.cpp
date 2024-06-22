// Copyright (c) TrajoptLib contributors

#include <trajopt/SwerveTrajectoryGenerator.hpp>

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
  path.WptConstraint(0, trajopt::LinearVelocityMaxMagnitudeConstraint{0.0});
  path.WptConstraint(1, trajopt::LinearVelocityMaxMagnitudeConstraint{0.0});
  path.ControlIntervalCounts({4});

  trajopt::SwerveTrajectoryGenerator generator{path};

  // SOLVE
  [[maybe_unused]]
  auto solution = generator.Generate(true);
}
