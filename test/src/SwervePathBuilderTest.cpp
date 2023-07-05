// Copyright (c) TrajoptLib contributors

#include <vector>

#include <gtest/gtest.h>

#include "trajopt/path/InitialGuessPoint.h"
#include "trajopt/path/SwervePathBuilder.h"

TEST(SwervePathBuilderTest, GenerateLinearInitialGuess) {
  using namespace trajopt;
  trajopt::SwervePathBuilder path;
  path.WptInitialGuessPoint(0, InitialGuessPoint{0.0, 0.0, 0.0});  // at 0

  path.SgmtInitialGuessPoints(
      0, {InitialGuessPoint{1.0, 0.0, 0.0},
          InitialGuessPoint{2.0, 0.0, 0.0}});  // from 0 to 1
  path.WptInitialGuessPoint(1, InitialGuessPoint{1.0, 0.0, 0.0});  // at 1

  path.WptInitialGuessPoint(2, InitialGuessPoint{5.0, 0.0, 0.0});  // at 2

  path.ControlIntervalCounts({3, 2});

  std::vector<double> result = path.CalculateInitialGuess().x;
  std::vector<double> expected = {0.0, 1.0, 2.0, 1.0, 3.0, 5.0};

  ASSERT_EQ(result, expected);
}
