// Copyright (c) TrajoptLib contributors

#include "path/Waypoint.h"

#include <array>
#include <variant>
#include <vector>

#include "constraint/Constraint.h"
#include "constraint/HeadingConstraint.h"
#include "constraint/ObstacleConstraint.h"
#include "constraint/PoseConstraint.h"
#include "constraint/TranslationConstraint.h"
#include "obstacle/Obstacle.h"
#include "path/InitialGuessPoint.h"
#include "set/ConeSet2d.h"
#include "set/EllipticalSet2d.h"
#include "set/LinearSet2d.h"
#include "set/RectangularSet2d.h"
#include "set/Set2d.h"

namespace helixtrajectory {

bool Waypoint::IsInitialWaypoint() const noexcept {
  return controlIntervalCount == 0;
}

bool Waypoint::IsValid() const noexcept {
  return (controlIntervalCount == 0 && initialGuessPoints.size() == 1) ||
         (initialGuessPoints.size() >= 1 &&
          initialGuessPoints.size() <= controlIntervalCount);
}

bool Waypoint::IsPositionStateKnown() const noexcept {
  // return waypointPositionConstraint.headingBound.IsExact()
  //         && waypointPositionConstraint.fieldRelativePositionBound.IsExact();
  return false;
}

bool Waypoint::IsStateKnown() const noexcept {
  return IsPositionStateKnown() && IsVelocityStateKnown();
}

bool Waypoint::IsSplitWaypoint() const noexcept {
  return IsInitialWaypoint() || IsStateKnown();
}

Waypoint::Waypoint(const std::vector<Constraint>& waypointConstraints,
                   const std::vector<Constraint>& segmentConstraints,
                   size_t controlIntervalCount,
                   const std::vector<InitialGuessPoint>& initialGuessPoints)
    : waypointConstraints(waypointConstraints),
      segmentConstraints(segmentConstraints),
      controlIntervalCount(controlIntervalCount),
      initialGuessPoints(initialGuessPoints) {}
}  // namespace helixtrajectory
