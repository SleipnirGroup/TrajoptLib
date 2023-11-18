// Copyright (c) TrajoptLib contributors

#pragma once

#include <utility>
#include <vector>

#include "optimization/OptiSys.h"
#include "trajopt/constraint/Constraint.h"
#include "trajopt/obstacle/Obstacle.h"
#include "trajopt/path/Path.h"
#include "trajopt/set/IntervalSet1d.h"
#include "trajopt/set/Set2d.h"
#include "trajopt/solution/Solution.h"

namespace trajopt {

template <ExprSys Expr>
std::pair<Expr, Expr> RotateVector(const Expr& x, const Expr& y,
                                   const Expr& theta);

template <ExprSys Expr>
std::pair<Expr, Expr> RotateConstantVector(double x, double y,
                                           const Expr& theta);

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
                     size_t sampIdx = 0);

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
void ApplyDiscreteTimeObjective(Opti& opti, std::vector<Expr>& dt,
                                const std::vector<size_t> N);

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
void ApplyIntervalSet1dConstraint(Opti& opti, const Expr& scalar,
                                  const IntervalSet1d& set1d);

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
void ApplySet2dConstraint(Opti& opti, const Expr& vectorX, const Expr& vectorY,
                          const Set2d& set2d);

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
std::vector<double> RowSolutionValue(const Opti& opti,
                                     const std::vector<Expr>& rowVector);

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
std::vector<std::vector<double>> MatrixSolutionValue(
    const Opti& opti, const std::vector<std::vector<Expr>>& matrix);

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
                                                           const Expr& theta,
                                                           double robotPointX,
                                                           double robotPointY);

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
void ApplyConstraint(Opti& opti, const Expr& x, const Expr& y,
                     const Expr& thetacos, const Expr& thetasin, const Constraint& constraint);

inline std::vector<double> Linspace(double startValue, double endValue,
                                    size_t numSamples);

inline Solution GenerateLinearInitialGuess(
    const std::vector<std::vector<InitialGuessPoint>>& initialGuessPoints,
    const std::vector<size_t> controlIntervalCounts);

template <typename Expr, typename Opti>
  requires OptiSys<Expr, Opti>
static void ApplyInitialGuess(Opti& opti, const Solution& solution,
                              std::vector<Expr>& x, std::vector<Expr>& y,
                              std::vector<Expr>& theta);
}  // namespace trajopt

#include "optimization/TrajoptUtil.inc"
