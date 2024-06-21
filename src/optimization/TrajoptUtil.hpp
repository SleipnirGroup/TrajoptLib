// Copyright (c) TrajoptLib contributors

#pragma once

#include <cmath>
#include <utility>
#include <variant>
#include <vector>

#include <sleipnir/optimization/OptimizationProblem.hpp>

#include "trajopt/constraint/Constraint.hpp"
#include "trajopt/constraint/LinePointConstraint.hpp"
#include "trajopt/constraint/PointLineConstraint.hpp"
#include "trajopt/constraint/PointPointConstraint.hpp"
#include "trajopt/constraint/TranslationConstraint.hpp"
#include "trajopt/path/InitialGuessPoint.hpp"
#include "trajopt/set/ConeSet2d.hpp"
#include "trajopt/set/EllipticalSet2d.hpp"
#include "trajopt/set/IntervalSet1d.hpp"
#include "trajopt/set/LinearSet2d.hpp"
#include "trajopt/set/RectangularSet2d.hpp"
#include "trajopt/set/Set2d.hpp"
#include "trajopt/solution/Solution.hpp"

namespace trajopt {

inline sleipnir::Variable fmax(const sleipnir::Variable& a,
                               const sleipnir::Variable& b) {
  return +0.5 * (1 + sleipnir::sign(b - a)) * (b - a) + a;
}

inline sleipnir::Variable fmin(const sleipnir::Variable& a,
                               const sleipnir::Variable& b) {
  return -0.5 * (1 + sleipnir::sign(b - a)) * (b - a) + b;
}

inline std::pair<sleipnir::Variable, sleipnir::Variable> RotateVector(
    const sleipnir::Variable& x, const sleipnir::Variable& y,
    const sleipnir::Variable& theta) {
  return {x * cos(theta) - y * sin(theta),   // NOLINT
          x * sin(theta) + y * cos(theta)};  // NOLINT
}

/**
 * @brief Get the index of an item in a decision variable array, given the
 * waypoint and sample indices and whether this array includes an entry
 * for the initial sample point ("dt" does not, "x" does).
 *
 * @param ctrlIntCnts the control interval counts of each segment, in order
 * @param wpIdx the waypoint index (1 + segment index)
 * @param sampIdx the sample index within the segment
 * @param hasInitialSamp whether this decision variable includes a value for
 * the initial sample
 * @return the index in the array
 */
inline size_t GetIdx(const std::vector<size_t>& N, size_t wptIdx,
                     size_t sampIdx = 0) {
  size_t idx = 0;
  if (wptIdx > 0) {
    ++idx;
  }
  for (size_t _wptIdx = 1; _wptIdx < wptIdx; ++_wptIdx) {
    idx += N.at(_wptIdx - 1);
  }
  idx += sampIdx;
  return idx;
}

inline void ApplyDiscreteTimeObjective(sleipnir::OptimizationProblem& problem,
                                       std::vector<sleipnir::Variable>& dt,
                                       const std::vector<size_t> N) {
  sleipnir::Variable T_tot = 0;
  for (size_t sgmtIdx = 0; sgmtIdx < N.size(); ++sgmtIdx) {
    auto& dt_sgmt = dt.at(sgmtIdx);
    auto N_sgmt = N.at(sgmtIdx);
    auto T_sgmt = dt_sgmt * static_cast<int>(N_sgmt);
    T_tot += T_sgmt;

    problem.SubjectTo(dt_sgmt >= 0);
    dt_sgmt.SetValue(5.0 / N_sgmt);
  }
  problem.Minimize(std::move(T_tot));
}

inline void ApplyIntervalSet1dConstraint(sleipnir::OptimizationProblem& problem,
                                         const sleipnir::Variable& scalar,
                                         const IntervalSet1d& set1d) {
  if (set1d.IsExact()) {
    problem.SubjectTo(scalar == set1d.lower);
  } else {
    if (set1d.IsLowerBounded()) {
      problem.SubjectTo(scalar >= set1d.lower);
    }
    if (set1d.IsUpperBounded()) {
      problem.SubjectTo(scalar <= set1d.upper);
    }
  }
}

inline void ApplySet2dConstraint(sleipnir::OptimizationProblem& problem,
                                 const sleipnir::Variable& vectorX,
                                 const sleipnir::Variable& vectorY,
                                 const Set2d& set2d) {
  if (std::holds_alternative<RectangularSet2d>(set2d)) {
    auto& rectangularSet2d = std::get<RectangularSet2d>(set2d);
    ApplyIntervalSet1dConstraint(problem, vectorX, rectangularSet2d.xBound);
    ApplyIntervalSet1dConstraint(problem, vectorY, rectangularSet2d.yBound);
  } else if (std::holds_alternative<LinearSet2d>(set2d)) {
    auto& linearSet2d = std::get<LinearSet2d>(set2d);
    double sinTheta = std::sin(linearSet2d.theta);
    double cosTheta = std::cos(linearSet2d.theta);
    problem.SubjectTo(vectorX * sinTheta == vectorY * cosTheta);
  } else if (std::holds_alternative<EllipticalSet2d>(set2d)) {
    auto& ellipticalSet2d = std::get<EllipticalSet2d>(set2d);
    auto scaledVectorXSquared = (vectorX * vectorX) / (ellipticalSet2d.xRadius *
                                                       ellipticalSet2d.xRadius);
    auto scaledVectorYSquared = (vectorY * vectorY) / (ellipticalSet2d.yRadius *
                                                       ellipticalSet2d.yRadius);
    auto lhs = scaledVectorXSquared + scaledVectorYSquared;
    using enum EllipticalSet2d::Direction;
    switch (ellipticalSet2d.direction) {
      case kInside:
        problem.SubjectTo(lhs <= 1.0);
        break;
      case kCentered:
        problem.SubjectTo(lhs == 1.0);
        break;
      case kOutside:
        problem.SubjectTo(lhs >= 1.0);
        break;
    }
  } else if (std::holds_alternative<ConeSet2d>(set2d)) {
    auto& coneSet2d = std::get<ConeSet2d>(set2d);
    problem.SubjectTo(vectorX * sin(coneSet2d.thetaBound.upper) >=  // NOLINT
                      vectorY * cos(coneSet2d.thetaBound.upper));   // NOLINT
    problem.SubjectTo(vectorX * sin(coneSet2d.thetaBound.lower) <=  // NOLINT
                      vectorY * cos(coneSet2d.thetaBound.lower));   // NOLINT
  }
}

inline std::vector<double> RowSolutionValue(
    std::vector<sleipnir::Variable>& rowVector) {
  std::vector<double> valueRowVector;
  valueRowVector.reserve(rowVector.size());
  for (auto& expression : rowVector) {
    valueRowVector.push_back(expression.Value());
  }
  return valueRowVector;
}

inline std::vector<std::vector<double>> MatrixSolutionValue(
    std::vector<std::vector<sleipnir::Variable>>& matrix) {
  std::vector<std::vector<double>> valueMatrix;
  valueMatrix.reserve(matrix.size());
  for (auto& row : matrix) {
    valueMatrix.push_back(RowSolutionValue(row));
  }
  return valueMatrix;
}

/**
 * @brief Get an expression for the position of a bumper corner relative
 * to the field coordinate system, given the robot's x-coordinate,
 * y-coordinate, and heading. The first row of the resulting matrix contains
 * the x-coordinate, and the second row contains the y-coordinate.
 *
 * @param x the instantaneous heading of the robot (scalar)
 * @param y the instantaneous heading of the robot (scalar)
 * @param theta the instantaneous heading of the robot (scalar)
 * @param bumperCorner the bumper corner to find the position for
 * @return the bumper corner 2 x 1 position vector
 */
inline static const std::pair<sleipnir::Variable, sleipnir::Variable>
SolveRobotPointPosition(const sleipnir::Variable& x,
                        const sleipnir::Variable& y,
                        const sleipnir::Variable& theta, double robotPointX,
                        double robotPointY) {
  std::pair<sleipnir::Variable, sleipnir::Variable> position{0.0, 0.0};
  if (robotPointX == 0.0 && robotPointY == 0.0) {
    position.first = x;
    position.second = y;
  } else {
    double cornerDiagonal = std::hypot(robotPointX, robotPointY);
    double cornerAngle = std::atan2(robotPointY, robotPointX);
    position.first = x + cornerDiagonal * cos(cornerAngle + theta);   // NOLINT
    position.second = y + cornerDiagonal * sin(cornerAngle + theta);  // NOLINT
  }
  return position;
}

// https://www.desmos.com/calculator/cqmc1tjtsv
template <typename LineNumberType, typename PointNumberType>
decltype(LineNumberType() + PointNumberType()) linePointDist(
    LineNumberType lineStartX, LineNumberType lineStartY,
    LineNumberType lineEndX, LineNumberType lineEndY, PointNumberType pointX,
    PointNumberType pointY) {
  auto lX = lineEndX - lineStartX;
  auto lY = lineEndY - lineStartY;
  auto vX = pointX - lineStartX;
  auto vY = pointY - lineStartY;
  auto dot = vX * lX + vY * lY;
  auto lNormSquared = lX * lX + lY * lY;
  auto t = dot / lNormSquared;
  auto tBounded = fmax(fmin(t, 1), 0);  // NOLINT
  auto iX = (1 - tBounded) * lineStartX + tBounded * lineEndX;
  auto iY = (1 - tBounded) * lineStartY + tBounded * lineEndY;
  auto distSquared =
      (iX - pointX) * (iX - pointX) + (iY - pointY) * (iY - pointY);
  return distSquared;
}

inline void ApplyConstraint(sleipnir::OptimizationProblem& problem,
                            const sleipnir::Variable& x,
                            const sleipnir::Variable& y,
                            const sleipnir::Variable& theta,
                            const Constraint& constraint) {
  if (std::holds_alternative<TranslationConstraint>(constraint)) {
    auto& translationConstraint = std::get<TranslationConstraint>(constraint);
    ApplySet2dConstraint(problem, x, y, translationConstraint.translationBound);
  } else if (std::holds_alternative<HeadingConstraint>(constraint)) {
    auto& headingConstraint = std::get<HeadingConstraint>(constraint);
    ApplyIntervalSet1dConstraint(problem, theta,
                                 headingConstraint.headingBound);
  } else if (std::holds_alternative<LinePointConstraint>(constraint)) {
    auto linePointConstraint = std::get<LinePointConstraint>(constraint);
    auto [lineStartX, lineStartY] = SolveRobotPointPosition(
        x, y, theta, linePointConstraint.robotLineStartX,
        linePointConstraint.robotLineStartY);
    auto [lineEndX, lineEndY] =
        SolveRobotPointPosition(x, y, theta, linePointConstraint.robotLineEndX,
                                linePointConstraint.robotLineEndY);
    double pointX = linePointConstraint.fieldPointX;
    double pointY = linePointConstraint.fieldPointY;
    auto dist = linePointDist(lineStartX, lineStartY, lineEndX, lineEndY,
                              pointX, pointY);
    auto distSquared = dist * dist;
    auto& distInterval = linePointConstraint.distance;
    auto distIntervalSquared = IntervalSet1d(std::pow(distInterval.lower, 2),
                                             std::pow(distInterval.upper, 2));
    ApplyIntervalSet1dConstraint(problem, distSquared, distIntervalSquared);
  } else if (std::holds_alternative<PointLineConstraint>(constraint)) {
    auto pointLineConstraint = std::get<PointLineConstraint>(constraint);
    double lineStartX = pointLineConstraint.fieldLineStartX;
    double lineStartY = pointLineConstraint.fieldLineStartY;
    double lineEndX = pointLineConstraint.fieldLineEndX;
    double lineEndY = pointLineConstraint.fieldLineEndY;
    auto [pointX, pointY] =
        SolveRobotPointPosition(x, y, theta, pointLineConstraint.robotPointX,
                                pointLineConstraint.robotPointY);
    auto dist = linePointDist(lineStartX, lineStartY, lineEndX, lineEndY,
                              pointX, pointY);
    auto distSquared = dist * dist;
    auto& distInterval = pointLineConstraint.distance;
    auto distIntervalSquared = IntervalSet1d(std::pow(distInterval.lower, 2),
                                             std::pow(distInterval.upper, 2));
    ApplyIntervalSet1dConstraint(problem, distSquared, distIntervalSquared);
  } else if (std::holds_alternative<PointPointConstraint>(constraint)) {
    auto pointPointConstraint = std::get<PointPointConstraint>(constraint);
    double robotPointX = pointPointConstraint.robotPointX;
    double robotPointY = pointPointConstraint.robotPointY;
    double fieldPointX = pointPointConstraint.fieldPointX;
    double fieldPointY = pointPointConstraint.fieldPointY;
    auto [bumperCornerX, bumperCornerY] =
        SolveRobotPointPosition(x, y, theta, robotPointX, robotPointY);
    auto dx = fieldPointX - bumperCornerX;
    auto dy = fieldPointY - bumperCornerY;
    auto pointDistSquared = dx * dx + dy * dy;
    IntervalSet1d distSquared = pointPointConstraint.distance;
    distSquared.lower *= distSquared.lower;
    distSquared.upper *= distSquared.upper;
    ApplyIntervalSet1dConstraint(problem, pointDistSquared, distSquared);
  }
}

inline std::vector<double> Linspace(double startValue, double endValue,
                                    size_t numSamples) {
  std::vector<double> result;
  double delta = (endValue - startValue) / numSamples;
  for (size_t index = 1; index <= numSamples; index++) {
    result.push_back(startValue + index * delta);
  }
  return result;
}

template <typename T>
inline void append_vector(std::vector<T>& base,
                          const std::vector<T>& newItems) {
  base.insert(base.end(), newItems.begin(), newItems.end());
}

inline Solution GenerateLinearInitialGuess(
    const std::vector<std::vector<InitialGuessPoint>>& initialGuessPoints,
    const std::vector<size_t> controlIntervalCounts) {
  size_t wptCnt = controlIntervalCounts.size() + 1;
  size_t sampTot = GetIdx(controlIntervalCounts, wptCnt, 0);
  Solution initialGuess{};
  initialGuess.x.reserve(sampTot);
  initialGuess.y.reserve(sampTot);
  initialGuess.theta.reserve(sampTot);
  initialGuess.dt.reserve(sampTot);
  initialGuess.x.push_back(initialGuessPoints.front().front().x);
  initialGuess.y.push_back(initialGuessPoints.front().front().y);
  initialGuess.theta.push_back(initialGuessPoints.front().front().heading);
  for (size_t i = 0; i < sampTot; i++) {
    initialGuess.dt.push_back((wptCnt * 5.0) / sampTot);
  }
  for (size_t wptIdx = 1; wptIdx < wptCnt; wptIdx++) {
    size_t N_sgmt = controlIntervalCounts.at(wptIdx - 1);
    size_t guessPointCount = initialGuessPoints.at(wptIdx).size();
    size_t N_guessSgmt = N_sgmt / guessPointCount;
    append_vector(
        initialGuess.x,
        Linspace(initialGuessPoints.at(wptIdx - 1).back().x,
                 initialGuessPoints.at(wptIdx).front().x, N_guessSgmt));
    append_vector(
        initialGuess.y,
        Linspace(initialGuessPoints.at(wptIdx - 1).back().y,
                 initialGuessPoints.at(wptIdx).front().y, N_guessSgmt));
    append_vector(
        initialGuess.theta,
        Linspace(initialGuessPoints.at(wptIdx - 1).back().heading,
                 initialGuessPoints.at(wptIdx).front().heading, N_guessSgmt));
    for (size_t guessPointIdx = 1; guessPointIdx < guessPointCount - 1;
         guessPointIdx++) {  // if three or more guess points
      append_vector(
          initialGuess.x,
          Linspace(initialGuessPoints.at(wptIdx).at(guessPointIdx - 1).x,
                   initialGuessPoints.at(wptIdx).at(guessPointIdx).x,
                   N_guessSgmt));
      append_vector(
          initialGuess.y,
          Linspace(initialGuessPoints.at(wptIdx).at(guessPointIdx - 1).y,
                   initialGuessPoints.at(wptIdx).at(guessPointIdx).y,
                   N_guessSgmt));
      append_vector(
          initialGuess.theta,
          Linspace(initialGuessPoints.at(wptIdx).at(guessPointIdx - 1).heading,
                   initialGuessPoints.at(wptIdx).at(guessPointIdx).heading,
                   N_guessSgmt));
    }
    if (guessPointCount > 1) {  // if two or more guess points
      size_t N_lastGuessSgmt = N_sgmt - (guessPointCount - 1) * N_guessSgmt;
      append_vector(
          initialGuess.x,
          Linspace(initialGuessPoints.at(wptIdx).at(guessPointCount - 2).x,
                   initialGuessPoints.at(wptIdx).back().x, N_lastGuessSgmt));
      append_vector(
          initialGuess.y,
          Linspace(initialGuessPoints.at(wptIdx).at(guessPointCount - 2).y,
                   initialGuessPoints.at(wptIdx).back().y, N_lastGuessSgmt));
      append_vector(
          initialGuess.theta,
          Linspace(
              initialGuessPoints.at(wptIdx).at(guessPointCount - 2).heading,
              initialGuessPoints.at(wptIdx).back().heading, N_lastGuessSgmt));
    }
  }
  return initialGuess;
}

inline void ApplyInitialGuess(
    const Solution& solution, std::vector<sleipnir::Variable>& x,
    std::vector<sleipnir::Variable>& y, std::vector<sleipnir::Variable>& theta,
    std::vector<sleipnir::Variable>& vx, std::vector<sleipnir::Variable>& vy,
    std::vector<sleipnir::Variable>& omega, std::vector<sleipnir::Variable>& ax,
    std::vector<sleipnir::Variable>& ay,
    std::vector<sleipnir::Variable>& alpha) {
  size_t sampleTotal = x.size();
  for (size_t sampleIndex = 0; sampleIndex < sampleTotal; sampleIndex++) {
    x[sampleIndex].SetValue(solution.x[sampleIndex]);
    y[sampleIndex].SetValue(solution.y[sampleIndex]);
    theta[sampleIndex].SetValue(solution.theta[sampleIndex]);
  }
  vx[0].SetValue(0.0);
  vy[0].SetValue(0.0);
  omega[0].SetValue(0.0);
  ax[0].SetValue(0.0);
  ay[0].SetValue(0.0);
  alpha[0].SetValue(0.0);
  for (size_t sampleIndex = 1; sampleIndex < sampleTotal; sampleIndex++) {
    vx[sampleIndex].SetValue(
        (solution.x[sampleIndex] - solution.x[sampleIndex - 1]) /
        solution.dt[sampleIndex]);
    vy[sampleIndex].SetValue(
        (solution.y[sampleIndex] - solution.y[sampleIndex - 1]) /
        solution.dt[sampleIndex]);
    omega[sampleIndex].SetValue(
        (solution.theta[sampleIndex] - solution.theta[sampleIndex - 1]) /
        solution.dt[sampleIndex]);

    ax[sampleIndex].SetValue(
        (vx[sampleIndex].Value() - vx[sampleIndex - 1].Value()) /
        solution.dt[sampleIndex]);
    ay[sampleIndex].SetValue(
        (vy[sampleIndex].Value() - vy[sampleIndex - 1].Value()) /
        solution.dt[sampleIndex]);
    alpha[sampleIndex].SetValue(
        (omega[sampleIndex].Value() - omega[sampleIndex - 1].Value()) /
        solution.dt[sampleIndex]);
  }
}

}  // namespace trajopt
