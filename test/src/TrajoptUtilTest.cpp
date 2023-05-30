// Copyright (c) TrajoptLib contributors

#include <gtest/gtest.h>
#include <vector>

#include "TestOpti.h"
#include "optimization/TrajectoryOptimizationProblem.h"
#include "path/InitialGuessPoint.h"
#include "set/IntervalSet1d.h"

TEST(TrajoptUtilTest, GetIdx) {
  auto result0 = trajopt::GetIdx({2, 3}, 0, 0);
  auto result1 = trajopt::GetIdx({2, 3}, 1, 1);
  auto result2 = trajopt::GetIdx({2, 3}, 2, 2);
  auto result3 = trajopt::GetIdx({2, 3}, 3, 0);
  EXPECT_EQ(result0, 0);
  EXPECT_EQ(result1, 2);
  EXPECT_EQ(result2, 5);
  EXPECT_EQ(result3, 6);
}

TEST(TrajoptUtilTest, ApplyDiscreteTimeObjective) {
  TestOpti opti;
  std::vector<double> dt = {-1, 3};
  std::vector<size_t> N = {20, 15};
  trajopt::ApplyDiscreteTimeObjective(opti, dt, N);
  EXPECT_EQ(opti.GetMinimizeObjective(), 25);
  EXPECT_TRUE(opti.IsViolating());
}

TEST(TrajoptUtilTest, ApplyIntervalSet1d) {
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
  for (auto test : std::initializer_list<IntervalSet1dTest>{
  //  {  set,   val,  isViolating}
      {case1,    1.0,       false},
      {case1,    3.0,       false},
      {case1,    4.0,        true},
      {case2,   -4.0,       false},
      {case2,   10.0,        true},
      {case3, negInf,       false},
      {case3,   -6.0,       false},
      {case3,    5.0,        true},
      {case4,    7.0,       false},
      {case4, posInf,       false},
      {case4,    5.0,        true}}) {
    TestOpti opti;
    trajopt::ApplyIntervalSet1dConstraint(opti, test.val, test.set);
    EXPECT_EQ(opti.IsViolating(), test.isViolating);
  } 
}

TEST(TrajoptUtilTest, Linspace) {
  auto result = trajopt::Linspace(0.0, 2.0, 2);
  std::vector correct{1.0, 2.0};
  EXPECT_EQ(result, correct);
}

TEST(TrajoptUtilTest, LinearInitialGuess) {
  std::vector<std::vector<trajopt::InitialGuessPoint>> initialGuessPoints{
    {{1, 0, 0}},
    {{2, 0, 0}, {3, 0, 0}},
    {{6, 0, 0}}
  };
  std::vector<size_t> controlIntervalCounts{2, 3};
  std::vector<double> expectedX{1, 2, 3, 4, 5, 6};
  auto result = trajopt::GenerateLinearInitialGuess(
      initialGuessPoints, controlIntervalCounts);
  EXPECT_EQ(expectedX, result.x);
}
