// Copyright (c) TrajoptLib contributors

#include "path/SwervePathBuilder.h"

#include <memory>
#include <stdexcept>
#include <vector>

#include "constraint/AngularVelocityConstraint.h"
#include "constraint/Constraint.h"
#include "constraint/HeadingConstraint.h"
#include "constraint/LinePointConstraint.h"
#include "constraint/PointLineConstraint.h"
#include "constraint/TranslationConstraint.h"
#include "constraint/holonomic/HolonomicVelocityConstraint.h"
#include "drivetrain/SwerveDrivetrain.h"
#include "obstacle/Obstacle.h"
#include "optimization/TrajectoryOptimizationProblem.h"
#include "path/InitialGuessPoint.h"
#include "path/Path.h"
#include "set/EllipticalSet2d.h"
#include "set/IntervalSet1d.h"
#include "set/LinearSet2d.h"
#include "set/RectangularSet2d.h"
#include "solution/Solution.h"

namespace trajopt {

const SwervePath& SwervePathBuilder::GetPath() const {
  return path;
}

void SwervePathBuilder::SetDrivetrain(SwerveDrivetrain drivetrain) {
  path.drivetrain = std::move(drivetrain);
}

void SwervePathBuilder::PoseWpt(size_t index, double x, double y, double heading) {
  NewWpts(index);
  path.waypoints.at(index).waypointConstraints.emplace_back(
    TranslationConstraint{RectangularSet2d{x, y}});
  path.waypoints.at(index).waypointConstraints.emplace_back(
    HeadingConstraint{heading});
  initialGuessPoints.at(index).emplace_back(InitialGuessPoint{x, y, heading});
}

void SwervePathBuilder::TranslationWpt(size_t index, double x, double y, double headingGuess) {
  NewWpts(index);
  path.waypoints.at(index).waypointConstraints.emplace_back(
    TranslationConstraint{RectangularSet2d{x, y}});
  initialGuessPoints.at(index).emplace_back(InitialGuessPoint{x, y, headingGuess});
}

void SwervePathBuilder::NewWpts(size_t finalIndex) {
  int64_t targetIdx = finalIndex;
  int64_t greatestIdx = path.waypoints.size() - 1;
  if (targetIdx > greatestIdx) {
    for (int64_t i = greatestIdx + 1; i <= targetIdx; i++) {
      path.waypoints.emplace_back(SwerveWaypoint{});
      initialGuessPoints.emplace_back(std::vector<InitialGuessPoint>{});
      controlIntervalCounts.push_back(i == 0 ? 0 : 40);
    }
  }
}

void SwervePathBuilder::AddInitialGuessPoint(size_t fromIdx, double x, double y, double heading) {
  NewWpts(fromIdx + 1);
  initialGuessPoints.at(fromIdx + 1).push_back(InitialGuessPoint{x, y, heading});
}

void SwervePathBuilder::WptVelocityDirection(size_t idx, double angle) {
  WptConstraint(idx, HolonomicVelocityConstraint{
      LinearSet2d{angle}, CoordinateSystem::kField});
}

void SwervePathBuilder::WptVelocityMagnitude(size_t idx, double v) {
  WptConstraint(idx, HolonomicVelocityConstraint{
      EllipticalSet2d::CircularSet2d(v), CoordinateSystem::kField});
}

void SwervePathBuilder::WptZeroVelocity(size_t idx) {
  WptConstraint(idx, HolonomicVelocityConstraint{
      RectangularSet2d{0.0, 0.0}, CoordinateSystem::kField});
}

void SwervePathBuilder::WptVelocityPolar(size_t idx, double vr, double vtheta) {
  WptConstraint(idx, HolonomicVelocityConstraint{
      RectangularSet2d::PolarExactSet2d(vr, vtheta), CoordinateSystem::kField});
}

void SwervePathBuilder::WptZeroAngularVelocity(size_t idx) {
  WptConstraint(idx, AngularVelocityConstraint{0.0});
}

void SwervePathBuilder::SgmtVelocityDirection(size_t fromIdx, size_t toIdx, double angle, bool includeWpts) {
  SgmtConstraint(fromIdx, toIdx, HolonomicVelocityConstraint{
      LinearSet2d{angle}, CoordinateSystem::kField}, includeWpts);
}

void SwervePathBuilder::SgmtVelocityMagnitude(size_t fromIdx, size_t toIdx, double v, bool includeWpts) {
  SgmtConstraint(fromIdx, toIdx,
    HolonomicVelocityConstraint{
      EllipticalSet2d{v, v, EllipticalSet2d::Direction::kInside}, CoordinateSystem::kField},
    includeWpts);
}

void SwervePathBuilder::SgmtZeroAngularVelocity(size_t fromIdx, size_t toIdx, bool includeWpts) {
  SgmtConstraint(fromIdx, toIdx,
    AngularVelocityConstraint{0.0},
    includeWpts);
}

void SwervePathBuilder::WptConstraint(size_t idx, const HolonomicConstraint& constraint) {
  NewWpts(idx);
  path.waypoints.at(idx).waypointConstraints.push_back(constraint);
}

void SwervePathBuilder::SgmtConstraint(size_t fromIdx, size_t toIdx, const HolonomicConstraint& constraint, bool includeWpts) {
  if (!(fromIdx < toIdx)) {
    throw std::runtime_error("fromIdx >= toIdx");
  }
  NewWpts(toIdx);
  if (includeWpts) {
    path.waypoints.at(fromIdx).waypointConstraints.push_back(constraint);
  }
  for (size_t idx = fromIdx + 1; idx <= toIdx; idx++) {
    if (includeWpts) {
      path.waypoints.at(fromIdx).waypointConstraints.push_back(constraint);
    }
    path.waypoints.at(fromIdx).segmentConstraints.push_back(constraint);
  }
}

void SwervePathBuilder::StartZeroVelocity() {
  WptZeroVelocity(0);
}

void SwervePathBuilder::AddBumpers(Bumpers&& newBumpers) {
  bumpers.emplace_back(std::move(newBumpers));
}

void SwervePathBuilder::WptObstacle(size_t idx, const Obstacle& obstacle) {
  for (auto& _bumpers : bumpers) {
    auto constraints = GetConstraintsForObstacle(_bumpers, obstacle);
    for (auto& constraint : constraints) {
      WptConstraint(idx, constraint);
    }
  }
}

void SwervePathBuilder::SgmtObstacle(size_t fromIdx, size_t toIdx,
    const Obstacle& obstacle, bool includeWpts) {
  for (auto& _bumpers : bumpers) {
    auto constraints = GetConstraintsForObstacle(_bumpers, obstacle);
    for (auto& constraint : constraints) {
      SgmtConstraint(fromIdx, toIdx, constraint, includeWpts);
    }
  }
}

void SwervePathBuilder::ControlIntervalCounts(std::vector<size_t>&& counts) {
  controlIntervalCounts = std::move(counts);
}

const std::vector<size_t>& SwervePathBuilder::GetControlIntervalCounts() const {
  return controlIntervalCounts;
}

inline void linspace(std::vector<double>& arry, size_t startIndex,
                     size_t endIndex, double startValue, double endValue) {
  size_t intervalCount = endIndex - startIndex;
  double delta = (endValue - startValue) / intervalCount;
  for (size_t index = 0; index < intervalCount; index++) {
    // arry[startIndex + index] = startValue + index * delta;
    arry.push_back(startValue + index * delta);
  }
}

Solution SwervePathBuilder::CalculateInitialGuess() const {
  size_t wptCnt = path.waypoints.size();
  size_t sampTot = GetIdx(controlIntervalCounts, wptCnt, 0);
  Solution initialGuess{};
  initialGuess.x.reserve(sampTot);
  initialGuess.y.reserve(sampTot);
  initialGuess.theta.reserve(sampTot);
  initialGuess.x.push_back(initialGuessPoints.at(0).at(0).x);
  initialGuess.y.push_back(initialGuessPoints.at(0).at(0).y);
  initialGuess.theta.push_back(initialGuessPoints.at(0).at(0).heading);
  size_t sampIdx = 1;
  for (size_t wptIdx = 1; wptIdx < wptCnt; wptIdx++) {
    auto& prevWptFinalInitialGuessPt = initialGuessPoints.at(wptIdx - 1).back();
    auto& currWptFirstInitialGuessPt = initialGuessPoints.at(wptIdx).front();
    
    size_t N_sgmt = controlIntervalCounts.at(wptIdx - 1);
    size_t guessPointCount = initialGuessPoints.at(wptIdx).size();
    size_t N_guessSgmt = N_sgmt / guessPointCount;
    linspace(initialGuess.x,
             sampIdx,
             sampIdx + N_guessSgmt,
             prevWptFinalInitialGuessPt.x,
             currWptFirstInitialGuessPt.x);
    linspace(initialGuess.y,
             sampIdx,
             sampIdx + N_guessSgmt,
             prevWptFinalInitialGuessPt.y,
             currWptFirstInitialGuessPt.y);
    linspace(initialGuess.theta,
             sampIdx,
             sampIdx + N_guessSgmt,
             prevWptFinalInitialGuessPt.heading,
             currWptFirstInitialGuessPt.heading);
    for (size_t guessPointIdx = 1; guessPointIdx < guessPointCount - 1;
         guessPointIdx++) {  // if three or more guess points
      size_t prevGuessPointSampIdx = sampIdx + guessPointIdx * N_guessSgmt;
      size_t guessPointSampleIndex = sampIdx + (guessPointIdx + 1) * N_guessSgmt;
      linspace(initialGuess.x, prevGuessPointSampIdx,
               guessPointSampleIndex,
               initialGuessPoints.at(wptIdx).at(guessPointIdx - 1).x,
               initialGuessPoints.at(wptIdx).at(guessPointIdx).x);
      linspace(initialGuess.y, prevGuessPointSampIdx,
               guessPointSampleIndex,
               initialGuessPoints.at(wptIdx).at(guessPointIdx - 1).y,
               initialGuessPoints.at(wptIdx).at(guessPointIdx).y);
      linspace(initialGuess.theta, prevGuessPointSampIdx,
               guessPointSampleIndex,
               initialGuessPoints.at(wptIdx).at(guessPointIdx - 1).heading,
               initialGuessPoints.at(wptIdx).at(guessPointIdx).heading);
    }
    if (guessPointCount > 1) {  // if two or more guess points
      size_t secondToLastGuessPointSampleIdx =
          sampIdx + (guessPointCount - 1) * N_guessSgmt;
      size_t finalGuessPointSampleIndex = sampIdx + N_sgmt;
      linspace(initialGuess.x, secondToLastGuessPointSampleIdx,
               finalGuessPointSampleIndex,
               initialGuessPoints.at(wptIdx).at(guessPointCount - 2).x,
               initialGuessPoints.at(wptIdx).at(guessPointCount - 1).x);
      linspace(initialGuess.y, secondToLastGuessPointSampleIdx,
               finalGuessPointSampleIndex,
               initialGuessPoints.at(wptIdx).at(guessPointCount - 2).y,
               initialGuessPoints.at(wptIdx).at(guessPointCount - 1).y);
      linspace(initialGuess.theta, secondToLastGuessPointSampleIdx,
               finalGuessPointSampleIndex,
               initialGuessPoints.at(wptIdx).at(guessPointCount - 2).heading,
               initialGuessPoints.at(wptIdx).at(guessPointCount - 1).heading);
    }
    sampIdx += N_sgmt;
  }
  return initialGuess;
}

std::vector<HolonomicConstraint> SwervePathBuilder::GetConstraintsForObstacle(
    const Bumpers& bumpers, const Obstacle& obstacle) {

  auto distConst =
      IntervalSet1d::LessThan(bumpers.safetyDistance + obstacle.safetyDistance);

  size_t bumperCornerCount = bumpers.points.size();
  size_t obstacleCornerCount = obstacle.points.size();
  if (bumperCornerCount == 1 && obstacleCornerCount == 1) {
    // if the bumpers and obstacle are only one point
    return {PointPointConstraint{bumpers.points.at(0).x, bumpers.points.at(0).y,
        obstacle.points.at(0).x, obstacle.points.at(0).y, distConst}};
  }

  std::vector<HolonomicConstraint> constraints;

  // robot bumper edge to obstacle point constraints
  for (const ObstaclePoint& obstaclePoint : obstacle.points) {
    // First apply constraint for all but last edge
    for (size_t bumperCornerIndex = 0;
        bumperCornerIndex < bumperCornerCount - 1; bumperCornerIndex++) {
      constraints.emplace_back(LinePointConstraint{
          bumpers.points.at(bumperCornerIndex).x,
          bumpers.points.at(bumperCornerIndex).y,
          bumpers.points.at(bumperCornerIndex + 1).x,
          bumpers.points.at(bumperCornerIndex + 1).y,
          obstaclePoint.x,
          obstaclePoint.y,
          distConst});
    }
    // apply to last edge: the edge connecting the last point to the first
    // must have at least three points to need this
    if (bumperCornerCount >= 3) {
      constraints.emplace_back(LinePointConstraint{
          bumpers.points.at(bumperCornerCount - 1).x,
          bumpers.points.at(bumperCornerCount - 1).y,
          bumpers.points.at(0).x,
          bumpers.points.at(0).y,
          obstaclePoint.x,
          obstaclePoint.y,
          distConst});
    }
  }

  // obstacle edge to bumper corner constraints
  for (const ObstaclePoint& bumperCorner : bumpers.points) {
    for (size_t obstacleCornerIndex = 0;
      obstacleCornerIndex < obstacleCornerCount - 1; obstacleCornerIndex++) {
      constraints.emplace_back(PointLineConstraint{
          bumperCorner.x,
          bumperCorner.y,
          obstacle.points.at(obstacleCornerIndex).x,
          obstacle.points.at(obstacleCornerIndex).y,
          obstacle.points.at(obstacleCornerIndex + 1).x,
          obstacle.points.at(obstacleCornerIndex + 1).y,
          distConst});
    }
    if (obstacleCornerCount >= 3) {
      constraints.emplace_back(PointLineConstraint{
          bumperCorner.x,
          bumperCorner.y,
          obstacle.points.at(bumperCornerCount - 1).x,
          obstacle.points.at(bumperCornerCount - 1).y,
          obstacle.points.at(0).x,
          obstacle.points.at(0).y,
          distConst});
    }
  }
  return constraints;
}
}  // namespace trajopt
