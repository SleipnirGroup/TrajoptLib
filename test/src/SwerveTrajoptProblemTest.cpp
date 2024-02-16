// Copyright (c) TrajoptLib contributors

#include <vector>

#include <gtest/gtest.h>
#include <trajopt/path/InitialGuessPoint.h>
#include <trajopt/set/IntervalSet1d.h>

#include "TestOpti.h"
#include "optimization/SwerveTrajoptUtil.h"
#include "optimization/TrajoptUtil.h"

TEST(TrajoptUtilTest, ApplyKinematicsConstraints) {
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
