// Copyright (c) TrajoptLib contributors

#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <trajopt/geometry/Translation2.hpp>
#include <trajopt/set/IntervalSet1d.hpp>

#include "optimization/SwerveTrajoptUtil.hpp"

TEST_CASE("SwerveTrajoptUtil - SolveNetForce()", "[SwerveTrajoptUtil]") {
  std::vector<sleipnir::Variable> Fx{4.0, 1.0, 3.0, 5.0};
  std::vector<sleipnir::Variable> Fy{0.0, 2.0, -5.0, 2.0};

  auto [Fx_net, Fy_net] = trajopt::SolveNetForce(Fx, Fy);

  CHECK(Fx_net.Value() == Catch::Approx(13.0).margin(1e-3));
  CHECK(Fy_net.Value() == Catch::Approx(-1.0).margin(1e-3));
}

TEST_CASE("SwerveTrajoptUtil - SolveNetTorque()", "[SwerveTrajoptUtil]") {
  constexpr double theta = 1.0;

  std::vector<sleipnir::Variable> Fx{-3.0, -5.0, -4.0, 2.0};
  std::vector<sleipnir::Variable> Fy{4.0, -5.0, -2.0, -4.0};
  std::vector<trajopt::SwerveModule> swerveModules = {
      {trajopt::Translation2d{1.0, 1.0}, 0.0, 0.0, 0.0},
      {trajopt::Translation2d{1.0, -1.0}, 0.0, 0.0, 0.0},
      {trajopt::Translation2d{-1.0, 1.0}, 0.0, 0.0, 0.0},
      {trajopt::Translation2d{-1.0, -1.0}, 0.0, 0.0, 0.0},
  };

  sleipnir::Variable tau_net = trajopt::SolveNetTorque(
      {std::cos(theta), std::sin(theta)}, Fx, Fy, swerveModules);

  CHECK(tau_net.Value() == Catch::Approx(0.6553658).margin(1e-3));
}
