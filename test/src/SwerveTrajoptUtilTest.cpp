// Copyright (c) TrajoptLib contributors

#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <trajopt/path/InitialGuessPoint.h>
#include <trajopt/set/IntervalSet1d.h>

#include "TestOpti.h"
#include "optimization/SwerveTrajoptUtil.h"
#include "optimization/TrajoptUtil.h"

TEST_CASE("SwerveTrajoptUtil - SolveNetForce()", "[SwerveTrajoptUtil]") {
  std::vector<double> Fx{4.0, 1.0, 3.0, 5.0};
  std::vector<double> Fy{0.0, 2.0, -5.0, 2.0};

  auto [Fx_net, Fy_net] = trajopt::SolveNetForce(Fx, Fy);

  CHECK(Fx_net == Catch::Approx(13.0).margin(1e-3));
  CHECK(Fy_net == Catch::Approx(-1.0).margin(1e-3));
}

TEST_CASE("SwerveTrajoptUtil - SolveNetTorque()", "[SwerveTrajoptUtil]") {
  constexpr double theta = 1.0;

  std::vector<double> Fx{-3.0, -5.0, -4.0, 2.0};
  std::vector<double> Fy{4.0, -5.0, -2.0, -4.0};
  std::vector<trajopt::SwerveModule> swerveModules = {
      {.x = 1.0,
       .y = 1.0,
       .wheelRadius = 0.0,
       .wheelMaxAngularVelocity = 0.0,
       .wheelMaxTorque = 0.0},
      {.x = 1.0,
       .y = -1.0,
       .wheelRadius = 0.0,
       .wheelMaxAngularVelocity = 0.0,
       .wheelMaxTorque = 0.0},
      {.x = -1.0,
       .y = 1.0,
       .wheelRadius = 0.0,
       .wheelMaxAngularVelocity = 0.0,
       .wheelMaxTorque = 0.0},
      {.x = -1.0,
       .y = -1.0,
       .wheelRadius = 0.0,
       .wheelMaxAngularVelocity = 0.0,
       .wheelMaxTorque = 0.0},
  };

  double tau_net = trajopt::SolveNetTorque(theta, Fx, Fy, swerveModules);

  CHECK(tau_net == Catch::Approx(0.6553658).margin(1e-3));
}

TEST_CASE("SwerveTrajoptUtil - ApplyKinematicsConstraints()",
          "[SwerveTrajoptUtil]") {
  TestOpti opti;
  //                         0      0      1      0      1      2     3
  std::vector<double> x{1.0, -1.0, 9.0, -19.0, 33.0, 101.0, 41.0};
  std::vector<double> v{1.0, -1.0, 5.0, -7.0, 13.0, 17.0, -15.0};
  std::vector<double> a{1.0, -1.0, 3.0, -3.0, 5.0, 1.0, -8.0};
  //                  dt{         2.0,   2.0,   4.0,   4.0,   4.0,   4.0}
  std::vector<double> dt{2.0, 4.0};
  std::vector<size_t> N{2, 4};

  trajopt::ApplyKinematicsConstraints(opti, x, x, x, v, v, v, a, a, a, dt, N);

  CHECK_FALSE(opti.IsViolating());
}

TEST_CASE("SwerveTrajoptUtil - ApplyDynamicsConstraints()",
          "[SwerveTrajoptUtil]") {
  TestOpti opti;
  constexpr double ax = 1.0;
  constexpr double ay = -2.0;
  constexpr double alpha = 9.0;
  constexpr double Fx_net = 45.0;
  constexpr double Fy_net = -90.0;
  constexpr double tau_net = 54.0;
  constexpr double mass = 45.0;
  constexpr double moi = 6.0;

  trajopt::ApplyDynamicsConstraints(opti, ax, ay, alpha, Fx_net, Fy_net,
                                    tau_net, mass, moi);

  CHECK_FALSE(opti.IsViolating());
}
