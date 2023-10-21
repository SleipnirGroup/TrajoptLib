// Copyright (c) TrajoptLib contributors

#include <vector>

#include <gtest/gtest.h>

#include "path/InitialGuessPoint.h"
#include "path/SwervePathBuilder.h"

TEST(ObstacleTest, GenerateLinearInitialGuess) {
  using namespace trajopt;
  trajopt::SwervePathBuilder path;
  path.SetDrivetrain(
    {
        45,
        6,
        [
            {
              x: 0.6,
              y: 0.6,
              wheel_radius: 0.04,
              wheel_max_angular_velocity: 70.0,
              wheel_max_torque: 2.0,
            },
            {
              x: 0.6,
              y: -0.6,
              wheel_radius: 0.04,
              wheel_max_angular_velocity: 70.0,
              wheel_max_torque: 2.0,
            },
            {
              x: -0.6,
              y: 0.6,
              wheel_radius: 0.04,
              wheel_max_angular_velocity: 70.0,
              wheel_max_torque: 2.0,
            },
            {
              x: -0.6,
              y: -0.6,
              wheel_radius: 0.04,
              wheel_max_angular_velocity: 70.0,
              wheel_max_torque: 2.0,
            }
        ]
    }
  );
  path.PoseWpt(0, 0.0, 0.0, 0.0);
  path.PoseWpt(1, 2.0, 2.0, 0.0);

  const length = 0.7;
  path.AddBumpers(trajopt::Bumpers{
    .safetyDistance = 0.1,
    .points = {
      {+length / 2, +width / 2},
      {-length / 2, +width / 2},
      {-length / 2, -width / 2},
      {+length / 2, -width / 2}
    }
  });

  path.SgmtObstacle(0, 1, trajopt::Obstacle{
    .safetyDistance = 1.0,
    .points = {{1.0, 1.0}}
  });

  path.ControlIntervalCounts({10});
  ASSERT_NO_THROW(trajopt::OptimalTrajectoryGenerator::Generate(path));
}
