// Copyright (c) TrajoptLib contributors

#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <trajopt/path/InitialGuessPoint.hpp>
#include <trajopt/set/IntervalSet1d.hpp>

#include "TestOpti.hpp"
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

TEST_CASE("TrajoptUtil - ApplyDiscreteTimeObjective()", "[TrajoptUtil]") {
  TestOpti opti;
  std::vector<double> dt = {-1, 3};
  std::vector<size_t> N = {20, 15};
  trajopt::ApplyDiscreteTimeObjective(opti, dt, N);
  CHECK(opti.GetMinimizeObjective() == 25);
}

TEST_CASE("TrajoptUtil - ApplyIntervalSet1d()", "[TrajoptUtil]") {
  auto case1 = trajopt::IntervalSet1d(1, 3);
  auto case2 = trajopt::IntervalSet1d(-4);
  auto case3 = trajopt::IntervalSet1d::LessThan(-6);
  auto case4 = trajopt::IntervalSet1d::GreaterThan(7);

  constexpr double negInf = -std::numeric_limits<double>::infinity();
  constexpr double posInf = +std::numeric_limits<double>::infinity();

  struct IntervalSet1dTest {
    trajopt::IntervalSet1d set;
    double val;
    bool isViolating;
  };

  // correct
  for (auto test :
       std::initializer_list<IntervalSet1dTest>{//  {  set,   val,  isViolating}
                                                {case1, 1.0, false},
                                                {case1, 3.0, false},
                                                {case1, 4.0, true},
                                                {case2, -4.0, false},
                                                {case2, 10.0, true},
                                                {case3, negInf, false},
                                                {case3, -6.0, false},
                                                {case3, 5.0, true},
                                                {case4, 7.0, false},
                                                {case4, posInf, false},
                                                {case4, 5.0, true}}) {
    TestOpti opti;
    trajopt::ApplyIntervalSet1dConstraint(opti, test.val, test.set);
  }
}

TEST_CASE("TrajoptUtil - Linspace()", "[TrajoptUtil]") {
  auto result = trajopt::Linspace(0.0, 2.0, 2);
  std::vector correct{1.0, 2.0};
  CHECK(result == correct);
}

TEST_CASE("TrajoptUtil - Linear initial guess", "[TrajoptUtil]") {
  std::vector<std::vector<trajopt::InitialGuessPoint>> initialGuessPoints{
      {{1, 0, 0}}, {{2, 0, 0}, {3, 0, 0}}, {{6, 0, 0}}};
  std::vector<size_t> controlIntervalCounts{2, 3};
  std::vector<double> expectedX{1, 2, 3, 4, 5, 6};
  auto result = trajopt::GenerateLinearInitialGuess(initialGuessPoints,
                                                    controlIntervalCounts);
  CHECK(expectedX == result.x);
}
