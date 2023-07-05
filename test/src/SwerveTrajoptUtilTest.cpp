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
  double tau_net = trajopt::SolveNetTorque(theta, Fx, Fy, swerveModules);
  EXPECT_NEAR(tau_net, 0.6553658, 0.001);
}

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

// Uncomment when fix is made for this function in future PR
// TEST(SwerveTrajoptUtilTest, ApplyDynamicsConstraints) {
//   TestOpti opti;
//   double ax = 1.0;
//   double ay = -2.0;
//   double alpha = 9.0;
//   double Fx_net = 45.0;
//   double Fy_net = -90.0;
//   double tau_net = 54.0;
//   double mass = 45.0;
//   double moi = 6.0;
//   trajopt::ApplyDynamicsConstraints(opti, ax, ay, alpha, Fx_net, Fy_net, tau_net, mass, moi);
//   EXPECT_FALSE(opti.IsViolating());
// }
