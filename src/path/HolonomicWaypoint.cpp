// Copyright (c) TrajoptLib contributors

#include "path/HolonomicWaypoint.h"

#include <vector>

#include "obstacle/Obstacle.h"
#include "path/InitialGuessPoint.h"
#include "path/Waypoint.h"

namespace trajopt {

HolonomicWaypoint::HolonomicWaypoint(
    std::vector<Constraint> waypointConstraints,
    std::vector<HolonomicConstraint> waypointHolonomicConstraints,
    std::vector<Constraint> segmentConstraints,
    std::vector<HolonomicConstraint> segmentHolonomicConstraints,
    size_t controlIntervalCount,
    std::vector<InitialGuessPoint> initialGuessPoints)
    : Waypoint(std::move(waypointConstraints), std::move(segmentConstraints),
               controlIntervalCount, std::move(initialGuessPoints)),
      waypointHolonomicConstraints(std::move(waypointHolonomicConstraints)),
      segmentHolonomicConstraints(std::move(segmentHolonomicConstraints)) {}

bool HolonomicWaypoint::IsVelocityStateKnown() const noexcept {
  // return waypointVelocityConstraint.angularVelocityBound.IsExact()
  //         && waypointVelocityConstraint.fieldRelativeVelocityBound.IsExact()
  //         && waypointVelocityConstraint.robotRelativeVelocityBound.IsExact();
  return false;
}

}  // namespace trajopt
