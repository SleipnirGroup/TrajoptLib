// Copyright (c) TrajoptLib contributors

#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <trajopt/set/IntervalSet1d.hpp>

#include "optimization/TrajoptUtil.hpp"

TEST_CASE("TrajoptUtil - GetIdx()", "[TrajoptUtil]") {
  auto result0 = trajopt::GetIdx({2, 3}, 0, 0);
  auto result1 = trajopt::GetIdx({2, 3}, 1, 1);
  auto result2 = trajopt::GetIdx({2, 3}, 2, 2);
  auto result3 = trajopt::GetIdx({2, 3}, 3, 0);
  CHECK(result0 == 0);
  CHECK(result1 == 2);
  CHECK(result2 == 5);
  CHECK(result3 == 6);
}

TEST_CASE("TrajoptUtil - Linspace()", "[TrajoptUtil]") {
  auto result = trajopt::Linspace(0.0, 2.0, 2);
  std::vector correct{1.0, 2.0};
  CHECK(result == correct);
}

TEST_CASE("TrajoptUtil - Linear initial guess", "[TrajoptUtil]") {
  std::vector<std::vector<trajopt::Pose2d>> initialGuessPoints{
      {{1, 0, 0}}, {{2, 0, 0}, {3, 0, 0}}, {{6, 0, 0}}};
  std::vector<size_t> controlIntervalCounts{2, 3};
  std::vector<double> expectedX{1, 2, 3, 4, 5, 6};
  auto result = trajopt::GenerateLinearInitialGuess(initialGuessPoints,
                                                    controlIntervalCounts);
  CHECK(expectedX == result.x);
}
