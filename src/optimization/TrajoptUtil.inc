// Copyright (c) TrajoptLib contributors

#pragma once

#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <numeric>
#include <utility>
#include <variant>
#include <vector>

#include "DebugOptions.h"
#include "optimization/TrajoptUtil.h"
#include "trajopt/TrajectoryGenerationException.h"
#include "trajopt/constraint/LinePointConstraint.h"
#include "trajopt/constraint/PointLineConstraint.h"
#include "trajopt/constraint/PointPointConstraint.h"
#include "trajopt/constraint/TranslationConstraint.h"
#include "trajopt/obstacle/Obstacle.h"
#include "trajopt/path/Path.h"
#include "trajopt/set/ConeSet2d.h"
#include "trajopt/set/EllipticalSet2d.h"
#include "trajopt/set/IntervalSet1d.h"
#include "trajopt/set/LinearSet2d.h"
#include "trajopt/set/RectangularSet2d.h"
#include "trajopt/set/Set2d.h"

namespace trajopt {

template <ExprSys Expr>
std::pair<Expr, Expr> RotateVector(const Expr& x, const Expr& y,
                                   const Expr& theta) {
  return {x * cos(theta) - y * sin(theta),   // NOLINT
          x * sin(theta) + y * cos(theta)};  // NOLINT
}

template <ExprSys Expr>
std::pair<Expr, Expr> RotateConstantVector(double x, double y,
                                           const Expr& theta) {
  return {x * cos(theta) - y * sin(theta),   // NOLINT
          x * sin(theta) + y * cos(theta)};  // NOLINT
}

size_t GetIdx(const std::vector<size_t>& N, size_t wptIdx, size_t sampIdx) {
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

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
void ApplyDiscreteTimeObjective(Opti& opti, std::vector<Expr>& dt,
                                const std::vector<size_t> N) {
  Expr T_tot = 0;
  for (size_t sgmtIdx = 0; sgmtIdx < N.size(); ++sgmtIdx) {
    auto& dt_sgmt = dt.at(sgmtIdx);
    auto N_sgmt = N.at(sgmtIdx);
    auto T_sgmt = dt_sgmt * N_sgmt;
    T_tot += T_sgmt;

    opti.SubjectTo(dt_sgmt >= 0);
    opti.SetInitial(dt_sgmt, 5.0 / N_sgmt);
  }
  opti.Minimize(std::move(T_tot));
}

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
void ApplyIntervalSet1dConstraint(Opti& opti, const Expr& scalar,
                                  const IntervalSet1d& set1d) {
  if (set1d.IsExact()) {
    opti.SubjectTo(scalar == set1d.lower);
  } else {
    if (set1d.IsLowerBounded()) {
      opti.SubjectTo(scalar >= set1d.lower);
    }
    if (set1d.IsUpperBounded()) {
      opti.SubjectTo(scalar <= set1d.upper);
    }
  }
}

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
void ApplySet2dConstraint(Opti& opti, const Expr& vectorX, const Expr& vectorY,
                          const Set2d& set2d) {
  if (std::holds_alternative<RectangularSet2d>(set2d)) {
    auto& rectangularSet2d = std::get<RectangularSet2d>(set2d);
    ApplyIntervalSet1dConstraint(opti, vectorX, rectangularSet2d.xBound);
    ApplyIntervalSet1dConstraint(opti, vectorY, rectangularSet2d.yBound);
  } else if (std::holds_alternative<LinearSet2d>(set2d)) {
    auto& linearSet2d = std::get<LinearSet2d>(set2d);
    double sinTheta = std::sin(linearSet2d.theta);
    double cosTheta = std::cos(linearSet2d.theta);
    opti.SubjectTo(vectorX * sinTheta == vectorY * cosTheta);
  } else if (std::holds_alternative<EllipticalSet2d>(set2d)) {
    auto& ellipticalSet2d = std::get<EllipticalSet2d>(set2d);
    double xRadius = ellipticalSet2d.xRadius;
    double yRadius = ellipticalSet2d.yRadius;
    if (xRadius != 0 && yRadius != 0) {
      auto scaledVectorXSquared = (vectorX * vectorX) / (xRadius * xRadius);
      auto scaledVectorYSquared = (vectorY * vectorY) / (yRadius * yRadius);
      auto lhs = scaledVectorXSquared + scaledVectorYSquared;
      using enum EllipticalSet2d::Direction;
      switch (ellipticalSet2d.direction) {
        case kInside:
          opti.SubjectTo(lhs <= 1.0);
          break;
        case kCentered:
          opti.SubjectTo(lhs == 1.0);
          break;
        case kOutside:
          opti.SubjectTo(lhs >= 1.0);
          break;
      }
    } else if (xRadius == 0 && yRadius != 0) {
      auto lhs = (vectorY * vectorY) / (yRadius * yRadius);
      using enum EllipticalSet2d::Direction;
      switch (ellipticalSet2d.direction) {
        case kInside:
          opti.SubjectTo(vectorX == 0);
          opti.SubjectTo(lhs <= 1.0);
          break;
        case kCentered:
          opti.SubjectTo(vectorX == 0);
          opti.SubjectTo(lhs == 1.0);
          break;
        case kOutside:
          opti.SubjectTo(vectorX != 0);
          opti.SubjectTo(lhs >= 1.0);
          break;
      }
    } else if (xRadius != 0 && yRadius == 0) {
      auto lhs = (vectorX * vectorX) / (xRadius * xRadius);
      using enum EllipticalSet2d::Direction;
      switch (ellipticalSet2d.direction) {
        case kInside:
          opti.SubjectTo(lhs <= 1.0);
          opti.SubjectTo(vectorY == 0);
          break;
        case kCentered:
          opti.SubjectTo(lhs == 1.0);
          opti.SubjectTo(vectorY == 0);
          break;
        case kOutside:
          opti.SubjectTo(lhs >= 1.0);
          opti.SubjectTo(vectorY != 0);
          break;
      }
    } else if (xRadius != 0 && yRadius != 0) {
      using enum EllipticalSet2d::Direction;
      switch (ellipticalSet2d.direction) {
        case kInside:
          opti.SubjectTo(vectorX == 0);
          opti.SubjectTo(vectorY == 0);
          break;
        case kCentered:
          opti.SubjectTo(vectorX == 0);
          opti.SubjectTo(vectorY == 0);
          break;
        case kOutside:
          opti.SubjectTo(vectorX != 0);
          opti.SubjectTo(vectorY != 0);
          break;
      }
    }
  } else if (std::holds_alternative<ConeSet2d>(set2d)) {
    auto& coneSet2d = std::get<ConeSet2d>(set2d);
    opti.SubjectTo(vectorX * sin(coneSet2d.thetaBound.upper) >=  // NOLINT
                   vectorY * cos(coneSet2d.thetaBound.upper));   // NOLINT
    opti.SubjectTo(vectorX * sin(coneSet2d.thetaBound.lower) <=  // NOLINT
                   vectorY * cos(coneSet2d.thetaBound.lower));   // NOLINT
  }
}

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
std::vector<double> RowSolutionValue(const Opti& opti,
                                     const std::vector<Expr>& rowVector) {
  std::vector<double> valueRowVector;
  valueRowVector.reserve(rowVector.size());
  for (auto& expression : rowVector) {
    valueRowVector.push_back(opti.SolutionValue(expression));
  }
  return valueRowVector;
}

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
std::vector<std::vector<double>> MatrixSolutionValue(
    const Opti& opti, const std::vector<std::vector<Expr>>& matrix) {
  std::vector<std::vector<double>> valueMatrix;
  valueMatrix.reserve(matrix.size());
  for (auto& row : matrix) {
    valueMatrix.push_back(RowSolutionValue(opti, row));
  }
  return valueMatrix;
}

template <typename Expr>
  requires ExprSys<Expr>
const std::pair<Expr, Expr> SolveRobotPointPosition(const Expr& x,
                                                    const Expr& y,
                                                    const Expr& theta,
                                                    double robotPointX,
                                                    double robotPointY) {
  std::pair<Expr, Expr> position{0.0, 0.0};
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

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
void ApplyConstraint(Opti& opti, const Expr& x, const Expr& y,
                     const Expr& theta, const Constraint& constraint) {
  if (std::holds_alternative<TranslationConstraint>(constraint)) {
    auto& translationConstraint = std::get<TranslationConstraint>(constraint);
    ApplySet2dConstraint(opti, x, y, translationConstraint.translationBound);
  } else if (std::holds_alternative<HeadingConstraint>(constraint)) {
    auto& headingConstraint = std::get<HeadingConstraint>(constraint);
    ApplyIntervalSet1dConstraint(opti, theta, headingConstraint.headingBound);
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
    ApplyIntervalSet1dConstraint(opti, distSquared, distIntervalSquared);
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
    ApplyIntervalSet1dConstraint(opti, distSquared, distIntervalSquared);
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
    ApplyIntervalSet1dConstraint(opti, pointDistSquared, distSquared);
  }
}

std::vector<double> Linspace(double startValue, double endValue,
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

Solution GenerateLinearInitialGuess(
    const std::vector<std::vector<InitialGuessPoint>>& initialGuessPoints,
    const std::vector<size_t> controlIntervalCounts) {
  size_t wptCnt = controlIntervalCounts.size() + 1;
  size_t sampTot = GetIdx(controlIntervalCounts, wptCnt, 0);
  Solution initialGuess{};
  initialGuess.x.reserve(sampTot);
  initialGuess.y.reserve(sampTot);
  initialGuess.theta.reserve(sampTot);
  initialGuess.x.push_back(initialGuessPoints.front().front().x);
  initialGuess.y.push_back(initialGuessPoints.front().front().y);
  initialGuess.theta.push_back(initialGuessPoints.front().front().heading);
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

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
void ApplyInitialGuess(Opti& opti, const Solution& solution,
                       std::vector<Expr>& x, std::vector<Expr>& y,
                       std::vector<Expr>& theta) {
  size_t sampleTotal = x.size();
  for (size_t sampleIndex = 0; sampleIndex < sampleTotal; sampleIndex++) {
    opti.SetInitial(x[sampleIndex], solution.x[sampleIndex]);
    opti.SetInitial(y[sampleIndex], solution.y[sampleIndex]);
    opti.SetInitial(theta[sampleIndex], solution.theta[sampleIndex]);
  }
}

}  // namespace trajopt
