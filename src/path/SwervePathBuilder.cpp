// Copyright (c) TrajoptLib contributors

#include "trajopt/path/SwervePathBuilder.h"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include "optimization/Cancellation.h"
#include "optimization/TrajoptUtil.h"
#include "trajopt/constraint/AngularVelocityConstraint.h"
#include "trajopt/constraint/Constraint.h"
#include "trajopt/constraint/HeadingConstraint.h"
#include "trajopt/constraint/LinePointConstraint.h"
#include "trajopt/constraint/PointLineConstraint.h"
#include "trajopt/constraint/TranslationConstraint.h"
#include "trajopt/constraint/holonomic/HolonomicVelocityConstraint.h"
#include "trajopt/drivetrain/SwerveDrivetrain.h"
#include "trajopt/obstacle/Obstacle.h"
#include "trajopt/path/InitialGuessPoint.h"
#include "trajopt/path/Path.h"
#include "trajopt/set/EllipticalSet2d.h"
#include "trajopt/set/IntervalSet1d.h"
#include "trajopt/set/LinearSet2d.h"
#include "trajopt/set/RectangularSet2d.h"
#include "trajopt/solution/Solution.h"

namespace trajopt {

const SwervePath& SwervePathBuilder::GetPath() const {
  return path;
}

void SwervePathBuilder::SetDrivetrain(SwerveDrivetrain drivetrain) {
  path.drivetrain = std::move(drivetrain);
}

void SwervePathBuilder::PoseWpt(size_t index, double x, double y,
                                double heading) {
  NewWpts(index);
  path.waypoints.at(index).waypointConstraints.emplace_back(
      TranslationConstraint{RectangularSet2d{x, y}});
  path.waypoints.at(index).waypointConstraints.emplace_back(
      HeadingConstraint{heading});
  WptInitialGuessPoint(index, InitialGuessPoint{x, y, heading});
}

void SwervePathBuilder::TranslationWpt(size_t index, double x, double y,
                                       double headingGuess) {
  NewWpts(index);
  path.waypoints.at(index).waypointConstraints.emplace_back(
      TranslationConstraint{RectangularSet2d{x, y}});
  WptInitialGuessPoint(index, InitialGuessPoint{x, y, headingGuess});
}

void SwervePathBuilder::WptInitialGuessPoint(
    size_t wptIdx, const InitialGuessPoint& poseGuess) {
  NewWpts(wptIdx);
  initialGuessPoints.at(wptIdx).back() = poseGuess;
}

void SwervePathBuilder::SgmtInitialGuessPoints(
    size_t fromIdx, const std::vector<InitialGuessPoint>& sgmtPoseGuess) {
  NewWpts(fromIdx + 1);
  std::vector<InitialGuessPoint>& toInitialGuessPoints =
      initialGuessPoints.at(fromIdx + 1);
  toInitialGuessPoints.insert(toInitialGuessPoints.begin(),
                              sgmtPoseGuess.begin(), sgmtPoseGuess.end());
}

void SwervePathBuilder::WptVelocityDirection(size_t idx, double angle) {
  WptConstraint(idx, HolonomicVelocityConstraint{LinearSet2d{angle},
                                                 CoordinateSystem::kField});
}

void SwervePathBuilder::WptVelocityMagnitude(size_t idx, double v) {
  if (std::abs(v) < 1e-4) {
    WptZeroVelocity(idx);
  } else {
    WptConstraint(idx,
                  HolonomicVelocityConstraint{EllipticalSet2d::CircularSet2d(v),
                                              CoordinateSystem::kField});
  }
}

void SwervePathBuilder::WptZeroVelocity(size_t idx) {
  WptConstraint(idx, HolonomicVelocityConstraint{RectangularSet2d{0.0, 0.0},
                                                 CoordinateSystem::kField});
}

void SwervePathBuilder::WptVelocityPolar(size_t idx, double vr, double vtheta) {
  WptConstraint(idx, HolonomicVelocityConstraint{
                         RectangularSet2d::PolarExactSet2d(vr, vtheta),
                         CoordinateSystem::kField});
}

void SwervePathBuilder::WptZeroAngularVelocity(size_t idx) {
  WptConstraint(idx, AngularVelocityConstraint{0.0});
}

void SwervePathBuilder::SgmtVelocityDirection(size_t fromIdx, size_t toIdx,
                                              double angle, bool includeWpts) {
  SgmtConstraint(
      fromIdx, toIdx,
      HolonomicVelocityConstraint{LinearSet2d{angle}, CoordinateSystem::kField},
      includeWpts);
}

void SwervePathBuilder::SgmtVelocityMagnitude(size_t fromIdx, size_t toIdx,
                                              double v, bool includeWpts) {
  Set2d set = EllipticalSet2d{v, v, EllipticalSet2d::Direction::kInside};
  if (std::abs(v) < 1e-4) {
    set = RectangularSet2d{0.0, 0.0};
  }
  SgmtConstraint(fromIdx, toIdx,
                 HolonomicVelocityConstraint{set, CoordinateSystem::kField},
                 includeWpts);
}

void SwervePathBuilder::SgmtZeroAngularVelocity(size_t fromIdx, size_t toIdx,
                                                bool includeWpts) {
  SgmtConstraint(fromIdx, toIdx, AngularVelocityConstraint{0.0}, includeWpts);
}

void SwervePathBuilder::WptConstraint(size_t idx,
                                      const HolonomicConstraint& constraint) {
  NewWpts(idx);
  path.waypoints.at(idx).waypointConstraints.push_back(constraint);
}

void SwervePathBuilder::SgmtConstraint(size_t fromIdx, size_t toIdx,
                                       const HolonomicConstraint& constraint,
                                       bool includeWpts) {
  if (!(fromIdx < toIdx)) {
    throw std::runtime_error("fromIdx >= toIdx");
  }
  NewWpts(toIdx);
  if (includeWpts) {
    path.waypoints.at(fromIdx).waypointConstraints.push_back(constraint);
  }
  for (size_t idx = fromIdx + 1; idx <= toIdx; idx++) {
    if (includeWpts) {
      path.waypoints.at(idx).waypointConstraints.push_back(constraint);
    }
    path.waypoints.at(idx).segmentConstraints.push_back(constraint);
  }
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
                                     const Obstacle& obstacle,
                                     bool includeWpts) {
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

Solution SwervePathBuilder::CalculateInitialGuess() const {
  return GenerateLinearInitialGuess(initialGuessPoints, controlIntervalCounts);
}

void SwervePathBuilder::NewWpts(size_t finalIndex) {
  int64_t targetIdx = finalIndex;
  int64_t greatestIdx = path.waypoints.size() - 1;
  if (targetIdx > greatestIdx) {
    for (int64_t i = greatestIdx + 1; i <= targetIdx; ++i) {
      path.waypoints.emplace_back(SwerveWaypoint{});
      initialGuessPoints.emplace_back(
          std::vector{InitialGuessPoint{0.0, 0.0, 0.0}});
      if (i != 0) {
        controlIntervalCounts.push_back(40);
      }
    }
  }
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
                                 obstacle.points.at(0).x,
                                 obstacle.points.at(0).y, distConst}};
  }

  std::vector<HolonomicConstraint> constraints;

  // robot bumper edge to obstacle point constraints
  for (auto& obstaclePoint : obstacle.points) {
    // First apply constraint for all but last edge
    for (size_t bumperCornerIndex = 0;
         bumperCornerIndex < bumperCornerCount - 1; bumperCornerIndex++) {
      constraints.emplace_back(
          LinePointConstraint{bumpers.points.at(bumperCornerIndex).x,
                              bumpers.points.at(bumperCornerIndex).y,
                              bumpers.points.at(bumperCornerIndex + 1).x,
                              bumpers.points.at(bumperCornerIndex + 1).y,
                              obstaclePoint.x, obstaclePoint.y, distConst});
    }
    // apply to last edge: the edge connecting the last point to the first
    // must have at least three points to need this
    if (bumperCornerCount >= 3) {
      constraints.emplace_back(LinePointConstraint{
          bumpers.points.at(bumperCornerCount - 1).x,
          bumpers.points.at(bumperCornerCount - 1).y, bumpers.points.at(0).x,
          bumpers.points.at(0).y, obstaclePoint.x, obstaclePoint.y, distConst});
    }
  }

  // obstacle edge to bumper corner constraints
  for (auto& bumperCorner : bumpers.points) {
    for (size_t obstacleCornerIndex = 0;
         obstacleCornerIndex < obstacleCornerCount - 1; obstacleCornerIndex++) {
      constraints.emplace_back(PointLineConstraint{
          bumperCorner.x, bumperCorner.y,
          obstacle.points.at(obstacleCornerIndex).x,
          obstacle.points.at(obstacleCornerIndex).y,
          obstacle.points.at(obstacleCornerIndex + 1).x,
          obstacle.points.at(obstacleCornerIndex + 1).y, distConst});
    }
    if (obstacleCornerCount >= 3) {
      constraints.emplace_back(PointLineConstraint{
          bumperCorner.x, bumperCorner.y,
          obstacle.points.at(bumperCornerCount - 1).x,
          obstacle.points.at(bumperCornerCount - 1).y, obstacle.points.at(0).x,
          obstacle.points.at(0).y, distConst});
    }
  }
  return constraints;
}

void SwervePathBuilder::CancelAll() {
  trajopt::GetCancellationFlag() = 1;
}
}  // namespace trajopt
