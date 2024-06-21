// Copyright (c) TrajoptLib contributors

#pragma once

#include <cmath>
#include <numbers>
#include <utility>
#include <variant>
#include <vector>

#include <sleipnir/autodiff/Variable.hpp>
#include <sleipnir/optimization/OptimizationProblem.hpp>

#include "trajopt/constraint/Constraint.hpp"
#include "trajopt/constraint/LinePointConstraint.hpp"
#include "trajopt/constraint/PointLineConstraint.hpp"
#include "trajopt/constraint/PointPointConstraint.hpp"
#include "trajopt/constraint/TranslationConstraint.hpp"
#include "trajopt/geometry/Pose2.hpp"
#include "trajopt/set/ConeSet2d.hpp"
#include "trajopt/set/EllipticalSet2d.hpp"
#include "trajopt/set/IntervalSet1d.hpp"
#include "trajopt/set/LinearSet2d.hpp"
#include "trajopt/set/RectangularSet2d.hpp"
#include "trajopt/set/Set2d.hpp"
#include "trajopt/solution/Solution.hpp"

namespace trajopt {

inline sleipnir::Variable max(const sleipnir::Variable& a,
                              const sleipnir::Variable& b) {
  return +0.5 * (1 + sleipnir::sign(b - a)) * (b - a) + a;
}

inline sleipnir::Variable min(const sleipnir::Variable& a,
                              const sleipnir::Variable& b) {
  return -0.5 * (1 + sleipnir::sign(b - a)) * (b - a) + b;
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
                                 const Translation2v& vector,
                                 const Set2d& set2d) {
  if (std::holds_alternative<RectangularSet2d>(set2d)) {
    auto& rectangularSet2d = std::get<RectangularSet2d>(set2d);

    ApplyIntervalSet1dConstraint(problem, vector.X(), rectangularSet2d.xBound);
    ApplyIntervalSet1dConstraint(problem, vector.Y(), rectangularSet2d.yBound);
  } else if (std::holds_alternative<LinearSet2d>(set2d)) {
    auto& linearSet2d = std::get<LinearSet2d>(set2d);
    Rotation2d theta{std::sin(linearSet2d.theta), std::cos(linearSet2d.theta)};

    problem.SubjectTo(vector.X() * theta.Sin() == vector.Y() * theta.Cos());
  } else if (std::holds_alternative<EllipticalSet2d>(set2d)) {
    auto& ellipticalSet2d = std::get<EllipticalSet2d>(set2d);

    auto scaledVectorXSquared =
        (vector.X() * vector.X()) /
        (ellipticalSet2d.xRadius * ellipticalSet2d.xRadius);
    auto scaledVectorYSquared =
        (vector.Y() * vector.Y()) /
        (ellipticalSet2d.yRadius * ellipticalSet2d.yRadius);
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

    problem.SubjectTo(vector.X() * sleipnir::sin(coneSet2d.thetaBound.upper) >=
                      vector.Y() * sleipnir::cos(coneSet2d.thetaBound.upper));
    problem.SubjectTo(vector.X() * sleipnir::sin(coneSet2d.thetaBound.lower) <=
                      vector.Y() * sleipnir::cos(coneSet2d.thetaBound.lower));
  } else if (std::holds_alternative<ManifoldIntervalSet2d>(set2d)) {
    auto& manifoldSet2d = std::get<ManifoldIntervalSet2d>(set2d);
    Translation2d mid{std::cos(manifoldSet2d.middle),
                      std::sin(manifoldSet2d.middle)};

    auto dot = mid.Dot(vector);
    // middle ⋅ vector ≥ std::cos(tolerance) ||vector||
    if (manifoldSet2d.tolerance == 0) {
      problem.SubjectTo(dot == sleipnir::hypot(vector.X(), vector.Y()));
    } else if (manifoldSet2d.tolerance < std::numbers::pi) {
      problem.SubjectTo(dot >= std::cos(manifoldSet2d.tolerance) *
                                   sleipnir::hypot(vector.X(), vector.Y()));
    }
    // wider tolerances permit the whole circle, so do nothing
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

template <typename T, typename U, typename V>
inline auto Lerp(T a, U b, V t) {
  return a + t * (b - a);
}

// https://www.desmos.com/calculator/cqmc1tjtsv
template <typename T, typename U>
decltype(auto) linePointDist(const Translation2<T>& lineStart,
                             const Translation2<T>& lineEnd,
                             const Translation2<U>& point) {
  using R = decltype(std::declval<T>() + std::declval<U>());

  Translation2<R> l{lineEnd.X() - lineStart.X(), lineEnd.Y() - lineStart.Y()};
  Translation2<R> v{point.X() - lineStart.X(), point.Y() - lineStart.Y()};

  auto t = v.Dot(l) / l.SquaredNorm();
  auto tBounded = max(min(t, 1), 0);  // NOLINT
  Translation2<R> i{Lerp(lineStart.X(), lineEnd.X(), tBounded),
                    Lerp(lineStart.Y(), lineEnd.Y(), tBounded)};
  return (i - point).SquaredNorm();
}

inline void ApplyConstraint(sleipnir::OptimizationProblem& problem,
                            const Pose2v& pose, const Constraint& constraint) {
  if (std::holds_alternative<TranslationConstraint>(constraint)) {
    auto& translationConstraint = std::get<TranslationConstraint>(constraint);

    ApplySet2dConstraint(problem, {pose.X(), pose.Y()},
                         translationConstraint.translationBound);
  } else if (std::holds_alternative<HeadingConstraint>(constraint)) {
    auto& headingConstraint = std::get<HeadingConstraint>(constraint);

    ApplySet2dConstraint(problem,
                         {pose.Rotation().Cos(), pose.Rotation().Sin()},
                         headingConstraint.headingBound);
  } else if (std::holds_alternative<LinePointConstraint>(constraint)) {
    auto linePointConstraint = std::get<LinePointConstraint>(constraint);
    const auto& [robotLineStart, robotLineEnd, fieldPoint, distance] =
        linePointConstraint;

    auto lineStart =
        pose.Translation() + robotLineStart.RotateBy(pose.Rotation());
    auto lineEnd = pose.Translation() + robotLineEnd.RotateBy(pose.Rotation());
    auto dist =
        linePointDist(lineStart, lineEnd, linePointConstraint.fieldPoint);
    auto distSquared = dist * dist;
    auto& distInterval = linePointConstraint.distance;
    auto distIntervalSquared = IntervalSet1d(std::pow(distInterval.lower, 2),
                                             std::pow(distInterval.upper, 2));
    ApplyIntervalSet1dConstraint(problem, distSquared, distIntervalSquared);
  } else if (std::holds_alternative<PointLineConstraint>(constraint)) {
    auto pointLineConstraint = std::get<PointLineConstraint>(constraint);
    const auto& [robotPoint, fieldLineStart, fieldLineEnd, distance] =
        pointLineConstraint;

    auto point = pose.Translation() + robotPoint.RotateBy(pose.Rotation());
    auto dist = linePointDist(fieldLineStart, fieldLineEnd, point);
    auto distSquared = dist * dist;
    auto& distInterval = pointLineConstraint.distance;
    auto distIntervalSquared = IntervalSet1d(std::pow(distInterval.lower, 2),
                                             std::pow(distInterval.upper, 2));
    ApplyIntervalSet1dConstraint(problem, distSquared, distIntervalSquared);
  } else if (std::holds_alternative<PointPointConstraint>(constraint)) {
    auto pointPointConstraint = std::get<PointPointConstraint>(constraint);
    const auto& [robotPoint, fieldPoint, distance] = pointPointConstraint;

    auto bumperCorner =
        pose.Translation() + robotPoint.RotateBy(pose.Rotation());
    auto dx = fieldPoint.X() - bumperCorner.X();
    auto dy = fieldPoint.Y() - bumperCorner.Y();
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

inline std::vector<double> AngleLinspace(double startValue, double endValue,
                                         size_t numSamples) {
  auto diff = endValue - startValue;
  // angleModulus
  const double modulus = 2 * std::numbers::pi;
  const double minimumInput = -std::numbers::pi;
  const double maximumInput = std::numbers::pi;
  // Wrap input if it's above the maximum input
  const double numMax = std::trunc((diff - minimumInput) / modulus);
  diff -= numMax * modulus;

  // Wrap input if it's below the minimum input
  const double numMin = std::trunc((diff - maximumInput) / modulus);
  diff -= numMin * modulus;

  return Linspace(startValue, startValue + diff, numSamples);
}

template <typename T>
inline void append_vector(std::vector<T>& base,
                          const std::vector<T>& newItems) {
  base.insert(base.end(), newItems.begin(), newItems.end());
}

inline Solution GenerateLinearInitialGuess(
    const std::vector<std::vector<Pose2d>>& initialGuessPoints,
    const std::vector<size_t> controlIntervalCounts) {
  size_t wptCnt = controlIntervalCounts.size() + 1;
  size_t sampTot = GetIdx(controlIntervalCounts, wptCnt, 0);

  Solution initialGuess;

  initialGuess.x.reserve(sampTot);
  initialGuess.y.reserve(sampTot);
  initialGuess.thetacos.reserve(sampTot);
  initialGuess.thetasin.reserve(sampTot);
  initialGuess.dt.reserve(sampTot);

  initialGuess.x.push_back(initialGuessPoints.front().front().X());
  initialGuess.y.push_back(initialGuessPoints.front().front().Y());
  initialGuess.thetacos.push_back(
      initialGuessPoints.front().front().Rotation().Cos());
  initialGuess.thetasin.push_back(
      initialGuessPoints.front().front().Rotation().Sin());

  for (size_t i = 0; i < sampTot; i++) {
    initialGuess.dt.push_back((wptCnt * 5.0) / sampTot);
  }

  for (size_t wptIdx = 1; wptIdx < wptCnt; wptIdx++) {
    size_t N_sgmt = controlIntervalCounts.at(wptIdx - 1);
    size_t guessPointCount = initialGuessPoints.at(wptIdx).size();
    size_t N_guessSgmt = N_sgmt / guessPointCount;
    append_vector(
        initialGuess.x,
        Linspace(initialGuessPoints.at(wptIdx - 1).back().X(),
                 initialGuessPoints.at(wptIdx).front().X(), N_guessSgmt));
    append_vector(
        initialGuess.y,
        Linspace(initialGuessPoints.at(wptIdx - 1).back().Y(),
                 initialGuessPoints.at(wptIdx).front().Y(), N_guessSgmt));
    auto wptThetas = AngleLinspace(
        initialGuessPoints.at(wptIdx - 1).back().Rotation().Radians(),
        initialGuessPoints.at(wptIdx).front().Rotation().Radians(),
        N_guessSgmt);
    for (auto theta : wptThetas) {
      initialGuess.thetacos.push_back(std::cos(theta));
      initialGuess.thetasin.push_back(std::sin(theta));
    }
    for (size_t guessPointIdx = 1; guessPointIdx < guessPointCount - 1;
         guessPointIdx++) {  // if three or more guess points
      append_vector(
          initialGuess.x,
          Linspace(initialGuessPoints.at(wptIdx).at(guessPointIdx - 1).X(),
                   initialGuessPoints.at(wptIdx).at(guessPointIdx).X(),
                   N_guessSgmt));
      append_vector(
          initialGuess.y,
          Linspace(initialGuessPoints.at(wptIdx).at(guessPointIdx - 1).Y(),
                   initialGuessPoints.at(wptIdx).at(guessPointIdx).Y(),
                   N_guessSgmt));
      auto guessThetas = AngleLinspace(
          initialGuessPoints.at(wptIdx)
              .at(guessPointIdx - 1)
              .Rotation()
              .Radians(),
          initialGuessPoints.at(wptIdx).at(guessPointIdx).Rotation().Radians(),
          N_guessSgmt);
      for (auto theta : guessThetas) {
        initialGuess.thetacos.push_back(std::cos(theta));
        initialGuess.thetasin.push_back(std::sin(theta));
      }
    }
    if (guessPointCount > 1) {  // if two or more guess points
      size_t N_lastGuessSgmt = N_sgmt - (guessPointCount - 1) * N_guessSgmt;
      append_vector(
          initialGuess.x,
          Linspace(initialGuessPoints.at(wptIdx).at(guessPointCount - 2).X(),
                   initialGuessPoints.at(wptIdx).back().X(), N_lastGuessSgmt));
      append_vector(
          initialGuess.y,
          Linspace(initialGuessPoints.at(wptIdx).at(guessPointCount - 2).Y(),
                   initialGuessPoints.at(wptIdx).back().Y(), N_lastGuessSgmt));
      auto lastThetas = AngleLinspace(
          initialGuessPoints.at(wptIdx)
              .at(guessPointCount - 2)
              .Rotation()
              .Radians(),
          initialGuessPoints.at(wptIdx).back().Rotation().Radians(),
          N_lastGuessSgmt);
      for (auto theta : lastThetas) {
        initialGuess.thetacos.push_back(std::cos(theta));
        initialGuess.thetasin.push_back(std::sin(theta));
      }
    }
  }

  return initialGuess;
}

inline void ApplyInitialGuess(const Solution& solution,
                              std::vector<sleipnir::Variable>& x,
                              std::vector<sleipnir::Variable>& y,
                              std::vector<sleipnir::Variable>& thetacos,
                              std::vector<sleipnir::Variable>& thetasin,
                              std::vector<sleipnir::Variable>& vx,
                              std::vector<sleipnir::Variable>& vy,
                              std::vector<sleipnir::Variable>& omega,
                              std::vector<sleipnir::Variable>& ax,
                              std::vector<sleipnir::Variable>& ay,
                              std::vector<sleipnir::Variable>& alpha) {
  size_t sampleTotal = x.size();
  for (size_t sampleIndex = 0; sampleIndex < sampleTotal; sampleIndex++) {
    x[sampleIndex].SetValue(solution.x[sampleIndex]);
    y[sampleIndex].SetValue(solution.y[sampleIndex]);
    thetacos[sampleIndex].SetValue(solution.thetacos[sampleIndex]);
    thetasin[sampleIndex].SetValue(solution.thetasin[sampleIndex]);
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

    double thetacos = solution.thetacos[sampleIndex];
    double thetasin = solution.thetasin[sampleIndex];
    double last_thetacos = solution.thetacos[sampleIndex - 1];
    double last_thetasin = solution.thetasin[sampleIndex - 1];

    omega[sampleIndex].SetValue(
        Rotation2d{thetacos, thetasin}
            .RotateBy(-Rotation2d{last_thetacos, last_thetasin})
            .Radians() /
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
