// Copyright (c) TrajoptLib contributors

#include "trajopt/path/SwervePathBuilder.hpp"

#include <cassert>
#include <cmath>
#include <utility>

#include "optimization/Cancellation.hpp"
#include "optimization/TrajoptUtil.hpp"
#include "trajopt/constraint/AngularVelocityConstraint.hpp"
#include "trajopt/constraint/Constraint.hpp"
#include "trajopt/constraint/HeadingConstraint.hpp"
#include "trajopt/constraint/LinePointConstraint.hpp"
#include "trajopt/constraint/PointLineConstraint.hpp"
#include "trajopt/constraint/TranslationConstraint.hpp"
#include "trajopt/constraint/holonomic/HolonomicVelocityConstraint.hpp"
#include "trajopt/obstacle/Obstacle.hpp"
#include "trajopt/set/EllipticalSet2d.hpp"
#include "trajopt/set/IntervalSet1d.hpp"
#include "trajopt/set/LinearSet2d.hpp"
#include "trajopt/set/RectangularSet2d.hpp"

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
  WptInitialGuessPoint(index, {x, y, {heading}});
}

void SwervePathBuilder::TranslationWpt(size_t index, double x, double y,
                                       double headingGuess) {
  NewWpts(index);
  path.waypoints.at(index).waypointConstraints.emplace_back(
      TranslationConstraint{RectangularSet2d{x, y}});
  WptInitialGuessPoint(index, {x, y, {headingGuess}});
}

void SwervePathBuilder::WptInitialGuessPoint(size_t wptIndex,
                                             const Pose2d& poseGuess) {
  NewWpts(wptIndex);
  initialGuessPoints.at(wptIndex).back() = poseGuess;
}

void SwervePathBuilder::SgmtInitialGuessPoints(
    size_t fromIndex, const std::vector<Pose2d>& sgmtPoseGuess) {
  NewWpts(fromIndex + 1);
  std::vector<Pose2d>& toInitialGuessPoints =
      initialGuessPoints.at(fromIndex + 1);
  toInitialGuessPoints.insert(toInitialGuessPoints.begin(),
                              sgmtPoseGuess.begin(), sgmtPoseGuess.end());
}

void SwervePathBuilder::WptVelocityDirection(size_t index, double angle) {
  WptConstraint(index, HolonomicVelocityConstraint{LinearSet2d{angle},
                                                   CoordinateSystem::kField});
}

void SwervePathBuilder::WptVelocityMagnitude(size_t index, double v) {
  if (std::abs(v) < 1e-4) {
    WptZeroVelocity(index);
  } else {
    WptConstraint(index,
                  HolonomicVelocityConstraint{EllipticalSet2d::CircularSet2d(v),
                                              CoordinateSystem::kField});
  }
}

void SwervePathBuilder::WptZeroVelocity(size_t index) {
  WptConstraint(index, HolonomicVelocityConstraint{RectangularSet2d{0.0, 0.0},
                                                   CoordinateSystem::kField});
}

void SwervePathBuilder::WptVelocityPolar(size_t index, double vr,
                                         double vtheta) {
  WptConstraint(index, HolonomicVelocityConstraint{
                           RectangularSet2d::PolarExactSet2d(vr, vtheta),
                           CoordinateSystem::kField});
}

void SwervePathBuilder::WptAngularVelocity(size_t index,
                                           double angular_velocity) {
  WptConstraint(index, AngularVelocityConstraint{angular_velocity});
}

void SwervePathBuilder::WptAngularVelocityMaxMagnitude(
    size_t index, double angular_velocity) {
  WptConstraint(index, AngularVelocityConstraint{
                           IntervalSet1d{-angular_velocity, angular_velocity}});
}

void SwervePathBuilder::WptZeroAngularVelocity(size_t index) {
  WptConstraint(index, AngularVelocityConstraint{0.0});
}

void SwervePathBuilder::SgmtVelocityDirection(size_t fromIndex, size_t toIndex,
                                              double angle, bool includeWpts) {
  SgmtConstraint(
      fromIndex, toIndex,
      HolonomicVelocityConstraint{LinearSet2d{angle}, CoordinateSystem::kField},
      includeWpts);
}

void SwervePathBuilder::SgmtVelocityMagnitude(size_t fromIndex, size_t toIndex,
                                              double v, bool includeWpts) {
  Set2d set = EllipticalSet2d{v, v, EllipticalSet2d::Direction::kInside};
  if (std::abs(v) < 1e-4) {
    set = RectangularSet2d{0.0, 0.0};
  }
  SgmtConstraint(fromIndex, toIndex,
                 HolonomicVelocityConstraint{set, CoordinateSystem::kField},
                 includeWpts);
}

void SwervePathBuilder::SgmtAngularVelocity(size_t fromIndex, size_t toIndex,
                                            double angular_velocity,
                                            bool includeWpts) {
  SgmtConstraint(fromIndex, toIndex,
                 AngularVelocityConstraint{angular_velocity}, includeWpts);
}

void SwervePathBuilder::SgmtAngularVelocityMaxMagnitude(size_t fromIndex,
                                                        size_t toIndex,
                                                        double angular_velocity,
                                                        bool includeWpts) {
  SgmtConstraint(fromIndex, toIndex,
                 AngularVelocityConstraint{
                     IntervalSet1d{-angular_velocity, angular_velocity}},
                 includeWpts);
}

void SwervePathBuilder::SgmtZeroAngularVelocity(size_t fromIndex,
                                                size_t toIndex,
                                                bool includeWpts) {
  SgmtConstraint(fromIndex, toIndex, AngularVelocityConstraint{0.0},
                 includeWpts);
}

void SwervePathBuilder::WptConstraint(size_t index,
                                      const HolonomicConstraint& constraint) {
  NewWpts(index);
  path.waypoints.at(index).waypointConstraints.push_back(constraint);
}

void SwervePathBuilder::SgmtConstraint(size_t fromIndex, size_t toIndex,
                                       const HolonomicConstraint& constraint,
                                       bool includeWpts) {
  assert(fromIndex < toIndex);

  NewWpts(toIndex);
  if (includeWpts) {
    path.waypoints.at(fromIndex).waypointConstraints.push_back(constraint);
  }
  for (size_t index = fromIndex + 1; index <= toIndex; index++) {
    if (includeWpts) {
      path.waypoints.at(index).waypointConstraints.push_back(constraint);
    }
    path.waypoints.at(index).segmentConstraints.push_back(constraint);
  }
}

void SwervePathBuilder::AddBumpers(Bumpers&& newBumpers) {
  bumpers.emplace_back(std::move(newBumpers));
}

void SwervePathBuilder::WptObstacle(size_t index, const Obstacle& obstacle) {
  for (auto& _bumpers : bumpers) {
    auto constraints = GetConstraintsForObstacle(_bumpers, obstacle);
    for (auto& constraint : constraints) {
      WptConstraint(index, constraint);
    }
  }
}

void SwervePathBuilder::SgmtObstacle(size_t fromIndex, size_t toIndex,
                                     const Obstacle& obstacle,
                                     bool includeWpts) {
  for (auto& _bumpers : bumpers) {
    auto constraints = GetConstraintsForObstacle(_bumpers, obstacle);
    for (auto& constraint : constraints) {
      SgmtConstraint(fromIndex, toIndex, constraint, includeWpts);
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
  int64_t targetIndex = finalIndex;
  int64_t greatestIndex = path.waypoints.size() - 1;
  if (targetIndex > greatestIndex) {
    for (int64_t i = greatestIndex + 1; i <= targetIndex; ++i) {
      path.waypoints.emplace_back(SwerveWaypoint{});
      initialGuessPoints.emplace_back(std::vector<Pose2d>{{0.0, 0.0, {0.0}}});
      if (i != 0) {
        controlIntervalCounts.push_back(40);
      }
    }
  }
}

std::vector<HolonomicConstraint> SwervePathBuilder::GetConstraintsForObstacle(
    const Bumpers& bumpers, const Obstacle& obstacle) {
  auto distConst = IntervalSet1d::GreaterThan(bumpers.safetyDistance +
                                              obstacle.safetyDistance);

  size_t bumperCornerCount = bumpers.points.size();
  size_t obstacleCornerCount = obstacle.points.size();
  if (bumperCornerCount == 1 && obstacleCornerCount == 1) {
    // if the bumpers and obstacle are only one point
    return {PointPointConstraint{bumpers.points.at(0), obstacle.points.at(0),
                                 distConst}};
  }

  std::vector<HolonomicConstraint> constraints;

  // robot bumper edge to obstacle point constraints
  for (auto& obstaclePoint : obstacle.points) {
    // First apply constraint for all but last edge
    for (size_t bumperCornerIndex = 0;
         bumperCornerIndex < bumperCornerCount - 1; bumperCornerIndex++) {
      constraints.emplace_back(LinePointConstraint{
          bumpers.points.at(bumperCornerIndex),
          bumpers.points.at(bumperCornerIndex + 1), obstaclePoint, distConst});
    }
    // apply to last edge: the edge connecting the last point to the first
    // must have at least three points to need this
    if (bumperCornerCount >= 3) {
      constraints.emplace_back(
          LinePointConstraint{bumpers.points.at(bumperCornerCount - 1),
                              bumpers.points.at(0), obstaclePoint, distConst});
    }
  }

  // obstacle edge to bumper corner constraints
  for (auto& bumperCorner : bumpers.points) {
    if (obstacleCornerCount > 1) {
      for (size_t obstacleCornerIndex = 0;
           obstacleCornerIndex < obstacleCornerCount - 1;
           obstacleCornerIndex++) {
        constraints.emplace_back(PointLineConstraint{
            bumperCorner, obstacle.points.at(obstacleCornerIndex),
            obstacle.points.at(obstacleCornerIndex + 1), distConst});
      }
      if (obstacleCornerCount >= 3) {
        constraints.emplace_back(PointLineConstraint{
            bumperCorner, obstacle.points.at(bumperCornerCount - 1),
            obstacle.points.at(0), distConst});
      }
    } else {
      constraints.emplace_back(
          PointPointConstraint{bumperCorner, obstacle.points.at(0), distConst});
    }
  }
  return constraints;
}

void SwervePathBuilder::AddIntermediateCallback(
    const std::function<void(SwerveSolution&, int64_t)> callback) {
  path.callbacks.push_back(callback);
}

void SwervePathBuilder::CancelAll() {
  trajopt::GetCancellationFlag() = 1;
}
}  // namespace trajopt
