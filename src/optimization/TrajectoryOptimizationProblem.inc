// Copyright (c) TrajoptLib contributors

#pragma once

#include <cmath>
#include <memory>
#include <exception>
#include <limits>
#include <numeric>
#include <variant>
#include <vector>

#include "DebugOptions.h"
#include "TrajectoryGenerationException.h"
#include "constraint/LinePointConstraint.h"
#include "constraint/PointLineConstraint.h"
#include "constraint/PointPointConstraint.h"
#include "constraint/TranslationConstraint.h"
#include "obstacle/Obstacle.h"
#include "optimization/TrajectoryOptimizationProblem.h"
#include "path/Path.h"
#include "set/ConeSet2d.h"
#include "set/EllipticalSet2d.h"
#include "set/IntervalSet1d.h"
#include "set/LinearSet2d.h"
#include "set/RectangularSet2d.h"
#include "set/Set2d.h"

namespace trajopt {

// template <typename Opti>
// TrajectoryOptimizationProblem<Opti>::TrajectoryOptimizationProblem(std::vector<size_t>&& ctrlIntCnts)
//       : ctrlIntCnts(std::move(ctrlIntCnts)),
//       wpCnt(TrajectoryOptimizationProblem<Opti>::ctrlIntCnts.size() + 1),
//       segCnt(wpCnt - 1),
//       ctrlIntTot(std::accumulate(ctrlIntCnts.begin(), ctrlIntCnts.end(), 0)),
//       sampTot(ctrlIntTot + 1) {
//   dt.reserve(ctrlIntCnts);
//   x.reserve(sampTot);
//   y.reserve(sampTot);
//   theta.reserve(sampTot);

//   for (size_t sampIdx = 0; sampIdx < sampTot; sampIdx++) {
//     x.push_back(opti.Variable());
//     y.push_back(opti.Variable());
//     theta.push_back(opti.Variable());
//   }

//   Expression totalT = 0;
//   for (size_t segIdx = 0; segIdx < segCnt; segIdx++) {
//     Expression segmentDt = opti.Variable();
//     size_t segCtrlIntCnt = ctrlIntCnts.at(segIdx);
//     for (size_t segIntIdx = 0; segIntIdx < segCtrlIntCnt; segIntIdx++) {
//       dt.push_back(segmentDt);
//     }
//     totalT += segCtrlIntCnt * segmentDt;
//     opti.SubjectTo(segmentDt >= 0);
//     opti.SetInitial(segmentDt, 5.0 / segCtrlIntCnt);
//   }
//   opti.Minimize(totalT);
// }

/// TEST CASES:
/// GetIdx({2, 3}, 0, 0) -> 0
/// GetIdx({2, 3}, 1, 1) -> 2
/// GetIdx({2, 3}, 2, 2) -> 5
/// GetIdx({2, 3}, 3, 0) -> 6
size_t GetIdx(const std::vector<size_t>& N,
              size_t wptIdx,
              size_t sampIdx) {
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

template<typename Expr, typename Opti> requires OptiSys<Expr, Opti>
void ApplyDiscreteTimeObjective(Opti& opti,
                                const std::vector<Expr>& dt,
                                const std::vector<size_t> N) {
  Expr T_tot = 0;
  for (size_t sgmtIdx = 0; sgmtIdx < dt.size(); ++sgmtIdx) {
    auto& dt_sgmt = dt.at(sgmtIdx);
    auto& N_sgmt = N.at(sgmtIdx);
    auto T_sgmt = dt_sgmt * N_sgmt;
    T_tot += T_sgmt;

    opti.SubjectTo(dt_sgmt >= 0);
    opti.SetInitial(dt_sgmt, 5.0 / N_sgmt);
  }
  opti.Minimize(T_tot);
}

template<typename Expr, typename Opti> requires OptiSys<Expr, Opti>
void ApplyIntervalSet1dConstraint(
    Opti& opti, const Expr& scalar, const IntervalSet1d& set1d) {
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

template<typename Expr, typename Opti> requires OptiSys<Expr, Opti>
void ApplySet2dConstraint(
    Opti& opti, const Expr& vectorX, const Expr& vectorY,
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
  }
}

template<typename Expr, typename Opti> requires OptiSys<Expr, Opti>
std::vector<double> RowSolutionValue(
    const Opti& opti, const std::vector<Expr>& rowVector) {
  std::vector<double> valueRowVector;
  valueRowVector.reserve(rowVector.size());
  for (auto& expression : rowVector) {
    valueRowVector.push_back(opti.SolutionValue(expression));
  }
  return valueRowVector;
}

template<typename Expr, typename Opti> requires OptiSys<Expr, Opti>
std::vector<std::vector<double>> MatrixSolutionValue(
    const Opti& opti, const std::vector<std::vector<Expr>>& matrix) {
  std::vector<std::vector<double>> valueMatrix;
  valueMatrix.reserve(matrix.size());
  for (auto& row : matrix) {
    valueMatrix.push_back(RowSolutionValue(opti, row));
  }
  return valueMatrix;
}

template<typename Expr> requires ExprSys<Expr>
const std::pair<Expr, Expr>
SolveRobotPointPosition(
    const Expr& x, const Expr& y, const Expr& theta,
    double robotPointX, double robotPointY) {
  std::pair<Expr, Expr> position{0.0, 0.0};
  if (robotPointX == 0.0 && robotPointY == 0.0) {
    position.first = x;
    position.second = y;
  } else {
    double cornerDiagonal = std::hypot(robotPointX, robotPointY);
    double cornerAngle = std::atan2(robotPointY, robotPointX);
    position.first = x + cornerDiagonal * cos(cornerAngle + theta);  // NOLINT
    position.second = y + cornerDiagonal * sin(cornerAngle + theta);  // NOLINT
  }
  return position;
}

// https://www.desmos.com/calculator/cqmc1tjtsv
template <typename Expression, typename LineNumberType,
          typename PointNumberType>
Expression linePointDist(LineNumberType lineStartX, LineNumberType lineStartY,
                         LineNumberType lineEndX, LineNumberType lineEndY,
                         PointNumberType pointX, PointNumberType pointY) {
  Expression lX = lineEndX - lineStartX;
  Expression lY = lineEndY - lineStartY;
  Expression vX = pointX - lineStartX;
  Expression vY = pointY - lineStartY;
  Expression dot = vX * lX + vY * lY;
  Expression lNormSquared = lX * lX + lY * lY;
  Expression t = dot / lNormSquared;
  // Expression tBounded = std::fmax(std::fmin(t, 1), 0);
  Expression tBounded = t;
  Expression iX = (1 - tBounded) * lineStartX + tBounded * lineEndX;
  Expression iY = (1 - tBounded) * lineStartY + tBounded * lineEndY;
  Expression distSquared =
      (iX - pointX) * (iX - pointX) + (iY - pointY) * (iY - pointY);
  return distSquared;
}

template<typename Expr, typename Opti> requires OptiSys<Expr, Opti>
void ApplyConstraint(
    Opti& opti, const Expr& x, const Expr& y,
    const Expr& theta,
    const Constraint& constraint) {
  if (std::holds_alternative<TranslationConstraint>(constraint)) {
    auto& translationConstraint = std::get<TranslationConstraint>(constraint);
    ApplySet2dConstraint(opti, x, y, translationConstraint.translationBound);
  } else if (std::holds_alternative<HeadingConstraint>(constraint)) {
    auto& headingConstraint = std::get<HeadingConstraint>(constraint);
    ApplyIntervalSet1dConstraint(opti, theta, headingConstraint.headingBound);
  } else if (std::holds_alternative<LinePointConstraint>(constraint)) {
    // pass
  } else if (std::holds_alternative<PointLineConstraint>(constraint)) {
    // pass
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

template<typename Expr, typename Opti> requires OptiSys<Expr, Opti>
void ApplyInitialGuess(
    Opti& opti, const Solution& solution,
    std::vector<Expr>& x,
    std::vector<Expr>& y,
    std::vector<Expr>& theta) {
  size_t sampleTotal = x.size();
  for (size_t sampleIndex = 0; sampleIndex < sampleTotal; sampleIndex++) {
    opti.SetInitial(x[sampleIndex], solution.x[sampleIndex]);
    opti.SetInitial(y[sampleIndex], solution.y[sampleIndex]);
    opti.SetInitial(theta[sampleIndex], solution.theta[sampleIndex]);
  }
}

}  // namespace trajopt
