#include "mylibrary.h"
#include <path/SwervePathBuilder.h>
#include <drivetrain/SwerveDrivetrain.h>
#include <OptimalTrajectoryGenerator.h>

double my_special_function(double input) {
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
  path.PoseWpt(1, input, 0.0, 0.0);
  path.WptZeroVelocity(0);
  path.WptZeroVelocity(1);
  path.ControlIntervalCounts({20});
  return OptimalTrajectoryGenerator::Generate(path).x.at(20);
}
