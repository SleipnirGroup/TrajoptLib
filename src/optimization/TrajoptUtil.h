// Copyright (c) TrajoptLib contributors

#pragma once

#include <cmath>
#include <utility>
#include <variant>
#include <vector>

#include "optimization/OptiSys.h"
#include "trajopt/constraint/Constraint.h"
#include "trajopt/constraint/LinePointConstraint.h"
#include "trajopt/constraint/PointLineConstraint.h"
#include "trajopt/constraint/PointPointConstraint.h"
#include "trajopt/constraint/TranslationConstraint.h"
#include "trajopt/path/InitialGuessPoint.h"
#include "trajopt/set/ConeSet2d.h"
#include "trajopt/set/EllipticalSet2d.h"
#include "trajopt/set/IntervalSet1d.h"
#include "trajopt/set/LinearSet2d.h"
#include "trajopt/set/RectangularSet2d.h"
#include "trajopt/set/Set2d.h"
#include "trajopt/solution/Solution.h"

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
    auto scaledVectorXSquared = (vectorX * vectorX) / (ellipticalSet2d.xRadius *
                                                       ellipticalSet2d.xRadius);
    auto scaledVectorYSquared = (vectorY * vectorY) / (ellipticalSet2d.yRadius *
                                                       ellipticalSet2d.yRadius);
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
  } else if (std::holds_alternative<ConeSet2d>(set2d)) {
    auto& coneSet2d = std::get<ConeSet2d>(set2d);
    opti.SubjectTo(vectorX * sin(coneSet2d.thetaBound.upper) >=  // NOLINT
                   vectorY * cos(coneSet2d.thetaBound.upper));   // NOLINT
    opti.SubjectTo(vectorX * sin(coneSet2d.thetaBound.lower) <=  // NOLINT
                   vectorY * cos(coneSet2d.thetaBound.lower));   // NOLINT
  } else if (std::holds_alternative<ManifoldIntervalSet2d>(set2d)) {
    auto& manifoldSet2d = std::get<ManifoldIntervalSet2d>(set2d);
    auto mid_cos = cos(manifoldSet2d.middle);
    auto mid_sin = sin(manifoldSet2d.middle);
    auto dot = (mid_cos * vectorX) + (mid_sin * vectorY);
    // vector dot middle >= cos(tolerance) * ||vector||
    if (manifoldSet2d.tolerance == 0) {
      opti.SubjectTo(dot == hypot(vectorX, vectorY));
    } else if (manifoldSet2d.tolerance < std::numbers::pi) {
      opti.SubjectTo(dot >= cos(manifoldSet2d.tolerance)*hypot(vectorX, vectorY));
    }
    // wider tolerances permit the whole circle, so do nothing
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
template <typename Expr>
  requires ExprSys<Expr>
static const std::pair<Expr, Expr> SolveRobotPointPosition(const Expr& x,
                                                           const Expr& y,
                                                           const Expr& thetacos,
                                                           const Expr& thetasin,
                                                           double robotPointX,
                                                           double robotPointY) {
  std::pair<Expr, Expr> position{0.0, 0.0};
  if (robotPointX == 0.0 && robotPointY == 0.0) {
    position.first = x;
    position.second = y;
  } else {
    double cornerDiagonal = std::hypot(robotPointX, robotPointY);
    double cornerAngle = std::atan2(robotPointY, robotPointX);
    position.first = x + cornerDiagonal * (thetacos * cos(cornerAngle) - thetasin* sin(cornerAngle));   // NOLINT
    position.second = y + cornerDiagonal * (thetacos * sin(cornerAngle) + thetasin * cos(cornerAngle));  // NOLINT
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
                     const Expr& thetacos, const Expr& thetasin, const Constraint& constraint) {
  if (std::holds_alternative<TranslationConstraint>(constraint)) {
    auto& translationConstraint = std::get<TranslationConstraint>(constraint);
    ApplySet2dConstraint(opti, x, y, translationConstraint.translationBound);
  } else if (std::holds_alternative<HeadingConstraint>(constraint)) {
    auto& headingConstraint = std::get<HeadingConstraint>(constraint);
    ApplySet2dConstraint(opti, thetacos, thetasin, headingConstraint.headingBound);
  } else if (std::holds_alternative<LinePointConstraint>(constraint)) {
    auto linePointConstraint = std::get<LinePointConstraint>(constraint);
    auto [lineStartX, lineStartY] = SolveRobotPointPosition(
        x, y, thetacos, thetasin, linePointConstraint.robotLineStartX,
        linePointConstraint.robotLineStartY);
    auto [lineEndX, lineEndY] =
        SolveRobotPointPosition(x, y, thetacos, thetasin, linePointConstraint.robotLineEndX,
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
        SolveRobotPointPosition(x, y, thetacos, thetasin, pointLineConstraint.robotPointX,
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
        SolveRobotPointPosition(x, y, thetacos, thetasin, robotPointX, robotPointY);
    auto dx = fieldPointX - bumperCornerX;
    auto dy = fieldPointY - bumperCornerY;
    auto pointDistSquared = dx * dx + dy * dy;
    IntervalSet1d distSquared = pointPointConstraint.distance;
    distSquared.lower *= distSquared.lower;
    distSquared.upper *= distSquared.upper;
    ApplyIntervalSet1dConstraint(opti, pointDistSquared, distSquared);
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

inline std::vector<double> AngleLinspace(double startValue, double endValue,
                             size_t numSamples) {
    auto diff = endValue - startValue;
    //angleModulus
   const double modulus = 2 * std::numbers::pi;
    const double minimumInput = -std::numbers::pi;
    const double maximumInput = std::numbers::pi;
    // Wrap input if it's above the maximum input
    const double numMax = std::trunc((diff - minimumInput) / modulus);
    diff -= numMax * modulus;

    // Wrap input if it's below the minimum input
    const double numMin = std::trunc((diff - maximumInput) / modulus);
    diff -= numMin * modulus;

    return Linspace(startValue,
                   startValue + diff, numSamples);
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
  initialGuess.thetacos.reserve(sampTot);
  initialGuess.thetasin.reserve(sampTot);
  initialGuess.dt.reserve(sampTot);
  initialGuess.x.push_back(initialGuessPoints.front().front().x);
  initialGuess.y.push_back(initialGuessPoints.front().front().y);
  initialGuess.thetacos.push_back(cos(initialGuessPoints.front().front().heading));
  initialGuess.thetasin.push_back(sin(initialGuessPoints.front().front().heading));
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
    auto wptTheta = AngleLinspace(initialGuessPoints.at(wptIdx - 1).back().heading,
                 initialGuessPoints.at(wptIdx).front().heading, N_guessSgmt);
    for (auto it = wptTheta.cbegin(); it != wptTheta.cend(); it++) {
      initialGuess.thetacos.push_back(cos(*it));
      initialGuess.thetasin.push_back(sin(*it));
    }
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
    auto guessTheta = AngleLinspace(initialGuessPoints.at(wptIdx).at(guessPointIdx - 1).heading,
                   initialGuessPoints.at(wptIdx).at(guessPointIdx).heading,
                   N_guessSgmt);
    for (auto it = guessTheta.cbegin(); it != guessTheta.cend(); it++) {
      initialGuess.thetacos.push_back(cos(*it));
      initialGuess.thetasin.push_back(sin(*it));
    }
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
      auto lastTheta = AngleLinspace(initialGuessPoints.at(wptIdx).at(guessPointCount - 2).heading,
                   initialGuessPoints.at(wptIdx).back().heading, N_lastGuessSgmt);
      for (auto it = lastTheta.cbegin(); it != lastTheta.cend(); it++) {
        initialGuess.thetacos.push_back(cos(*it));
        initialGuess.thetasin.push_back(sin(*it));
      }
    }
  }
  return initialGuess;
}

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
void ApplyInitialGuess(Opti& opti, const Solution& solution,
                       std::vector<Expr>& x, std::vector<Expr>& y,
                       std::vector<Expr>& thetacos, std::vector<Expr>& thetasin, std::vector<Expr>& vx,
                       std::vector<Expr>& vy, std::vector<Expr>& omega,
                       std::vector<Expr>& ax, std::vector<Expr>& ay,
                       std::vector<Expr>& alpha) {
  size_t sampleTotal = x.size();
  for (size_t sampleIndex = 0; sampleIndex < sampleTotal; sampleIndex++) {
    opti.SetInitial(x[sampleIndex], solution.x[sampleIndex]);
    opti.SetInitial(y[sampleIndex], solution.y[sampleIndex]);
    opti.SetInitial(thetacos[sampleIndex], solution.thetacos[sampleIndex]);
    opti.SetInitial(thetasin[sampleIndex], solution.thetasin[sampleIndex]);
  }
  opti.SetInitial(vx[0], 0.0);
  opti.SetInitial(vy[0], 0.0);
  opti.SetInitial(omega[0], 0.0);
  opti.SetInitial(ax[0], 0.0);
  opti.SetInitial(ay[0], 0.0);
  opti.SetInitial(alpha[0], 0.0);
  for (size_t sampleIndex = 1; sampleIndex < sampleTotal; sampleIndex++) {
    opti.SetInitial(vx[sampleIndex],
                    (solution.x[sampleIndex] - solution.x[sampleIndex - 1]) /
                        solution.dt[sampleIndex]);
    opti.SetInitial(vy[sampleIndex],
                    (solution.y[sampleIndex] - solution.y[sampleIndex - 1]) /
                        solution.dt[sampleIndex]);
    double thetacos = solution.thetacos[sampleIndex];
    double thetasin = solution.thetasin[sampleIndex];
    double last_thetacos = solution.thetacos[sampleIndex-1];
    double last_thetasin = solution.thetasin[sampleIndex-1];
    // rotate <thetacos, thetasin> by the angle of -<last_thetacos, last_thetasin>
    // cos(-last)=last_thetacos, sin(-last) = -last_thetasin
    double diffcos = (thetacos * last_thetacos) + (thetasin * last_thetasin);
    double diffsin = (thetacos * -last_thetasin) + (thetasin*last_thetacos);
    opti.SetInitial(omega[sampleIndex], (std::atan2(diffsin, diffcos)) /
                                            solution.dt[sampleIndex]);

    opti.SetInitial(ax[sampleIndex], (opti.SolutionValue(vx[sampleIndex]) -
                                      opti.SolutionValue(vx[sampleIndex - 1])) /
                                         solution.dt[sampleIndex]);
    opti.SetInitial(ay[sampleIndex], (opti.SolutionValue(vy[sampleIndex]) -
                                      opti.SolutionValue(vy[sampleIndex - 1])) /
                                         solution.dt[sampleIndex]);
    opti.SetInitial(alpha[sampleIndex],
                    (opti.SolutionValue(omega[sampleIndex]) -
                     opti.SolutionValue(omega[sampleIndex - 1])) /
                        solution.dt[sampleIndex]);
  }
}

}  // namespace trajopt
