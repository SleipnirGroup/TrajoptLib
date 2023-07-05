// Copyright (c) TrajoptLib contributors

#include <array>
#include <numeric>
#include <vector>

#include <gtest/gtest.h>

#include "TestOpti.h"
#include "optimization/SwerveTrajoptUtil.h"
#include "optimization/TrajoptUtil.h"
#include "trajopt/path/InitialGuessPoint.h"
#include "trajopt/set/IntervalSet1d.h"

TEST(SwerveTrajoptUtilTest, ApplyKinematicsConstraints) {
  TestOpti opti;
  //                         0      0      1      0      1      2     3
  std::vector<double> x{1.0, -1.0, 9.0, -19.0, 33.0, 101.0, 41.0};
  std::vector<double> v{1.0, -1.0, 5.0, -7.0, 13.0, 17.0, -15.0};
  std::vector<double> a{1.0, -1.0, 3.0, -3.0, 5.0, 1.0, -8.0};
  //                  dt{         2.0,   2.0,   4.0,   4.0,   4.0,   4.0}
  std::vector<double> dt{2.0, 4.0};
  std::vector<size_t> N{2, 4};
  trajopt::ApplyKinematicsConstraints(opti, x, x, x, v, v, v, a, a, a, dt, N);
  EXPECT_FALSE(opti.IsViolating());
}

TEST(SwerveTrajoptUtilTest, SolveNetForce) {
  std::vector<double> Fx{4.0, 1.0, 3.0, 5.0};
  std::vector<double> Fy{0.0, 2.0, -5.0, 2.0};
  auto [Fx_net, Fy_net] = trajopt::SolveNetForce(Fx, Fy);
  EXPECT_DOUBLE_EQ(Fx_net, 13.0);
  EXPECT_DOUBLE_EQ(Fy_net, -1.0);
}

TEST(SwerveTrajoptUtilTest, SolveNetTorque) {
  double theta = 1.0;
  std::vector<double> Fx{-3.0, -5.0, -4.0, 2.0};
  std::vector<double> Fy{4.0, -5.0, -2.0, -4.0};
  std::vector<trajopt::SwerveModule> swerveModules = {
    {.x =  1.0, .y =  1.0, .wheelRadius = 0.0, .wheelMaxAngularVelocity = 0.0,
        .wheelMaxTorque = 0.0},
    {.x =  1.0, .y = -1.0, .wheelRadius = 0.0, .wheelMaxAngularVelocity = 0.0,
        .wheelMaxTorque = 0.0},
    {.x = -1.0, .y =  1.0, .wheelRadius = 0.0, .wheelMaxAngularVelocity = 0.0,
        .wheelMaxTorque = 0.0},
    {.x = -1.0, .y = -1.0, .wheelRadius = 0.0, .wheelMaxAngularVelocity = 0.0,
        .wheelMaxTorque = 0.0},
  };
  std::array<double, 4> tau = {2.94064515627, -5.40302305868, 1.55887186559, 1.55887186559};
  double tau_net = std::accumulate(tau.begin(), tau.end(), 0.0);
  double solved_tau_net = trajopt::SolveNetTorque(theta, Fx, Fy, swerveModules);
  EXPECT_NEAR(solved_tau_net, tau_net, 0.001);
}
