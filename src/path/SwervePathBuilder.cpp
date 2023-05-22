// Copyright (c) TrajoptLib contributors

#include "path/SwervePathBuilder.h"

#include <memory>
#include <stdexcept>
#include <vector>

#include "constraint/AngularVelocityConstraint.h"
#include "constraint/Constraint.h"
#include "constraint/HeadingConstraint.h"
#include "constraint/TranslationConstraint.h"
#include "constraint/holonomic/HolonomicVelocityConstraint.h"
#include "drivetrain/SwerveDrivetrain.h"
#include "obstacle/Obstacle.h"
#include "path/InitialGuessPoint.h"
#include "path/Path.h"
#include "set/EllipticalSet2d.h"
#include "set/LinearSet2d.h"
#include "set/RectangularSet2d.h"

namespace trajopt {

const SwervePath& SwervePathBuilder::GetPath() const {
  return path;
}

void SwervePathBuilder::SetDrivetrain(SwerveDrivetrain drivetrain) {
  if (path.waypoints.empty()) {
    path.drivetrain = std::move(drivetrain);
  } else {
    path.waypoints.back().segmentDrivetrain = std::move(drivetrain);
  }
}

void SwervePathBuilder::PoseWpt(size_t index, double x, double y, double heading) {
  NewWpts(index);
  path.waypoints.at(index).waypointConstraints.emplace_back(
    TranslationConstraint{RectangularSet2d{x, y}});
  path.waypoints.at(index).waypointConstraints.emplace_back(
    HeadingConstraint{heading});
  initalGuessPoints.at(index).emplace_back(InitialGuessPoint{x, y, heading});
}

void SwervePathBuilder::TranslationWpt(size_t index, double x, double y, double headingGuess) {
  NewWpts(index);
  path.waypoints.at(index).waypointConstraints.emplace_back(
    TranslationConstraint{RectangularSet2d{x, y}});
  initalGuessPoints.at(index).emplace_back(InitialGuessPoint{x, y, headingGuess});
}

void SwervePathBuilder::NewWpts(size_t finalIndex) {
  int64_t targetIdx = finalIndex;
  int64_t greatestIdx = path.waypoints.size() - 1;
  if (targetIdx > greatestIdx) {
    for (int64_t i = greatestIdx + 1; i <= targetIdx; i++) {
      path.waypoints.emplace_back(SwerveWaypoint{});
      initalGuessPoints.emplace_back(std::vector<InitialGuessPoint>{});
      controlIntervalCounts.push_back(i == 0 ? 0 : 40);
    }
  }
}

void SwervePathBuilder::AddInitialGuessPoint(size_t fromIdx, double x, double y, double heading) {
  NewWpts(fromIdx + 1);
  initalGuessPoints.at(fromIdx + 1).push_back(InitialGuessPoint{x, y, heading});
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
}  // namespace trajopt
