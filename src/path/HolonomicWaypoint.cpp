// Copyright (c) TrajoptLib contributors

#include "path/HolonomicWaypoint.h"

#include <vector>

#include "obstacle/Obstacle.h"
#include "path/InitialGuessPoint.h"
#include "path/Waypoint.h"

namespace trajopt {

HolonomicWaypoint::HolonomicWaypoint(
    const std::vector<Constraint>& waypointConstraints,
    const std::vector<HolonomicConstraint>& waypointHolonomicConstraints,
    const std::vector<Constraint>& segmentConstraints,
    const std::vector<HolonomicConstraint>& segmentHolonomicConstraints,
    size_t controlIntervalCount,
    const std::vector<InitialGuessPoint>& initialGuessPoints)
    : Waypoint(waypointConstraints, segmentConstraints, controlIntervalCount,
               initialGuessPoints),
      waypointHolonomicConstraints(waypointHolonomicConstraints),
      segmentHolonomicConstraints(segmentHolonomicConstraints) {}

bool HolonomicWaypoint::IsVelocityStateKnown() const noexcept {
  // return waypointVelocityConstraint.angularVelocityBound.IsExact()
  //         && waypointVelocityConstraint.fieldRelativeVelocityBound.IsExact()
  //         && waypointVelocityConstraint.robotRelativeVelocityBound.IsExact();
  return false;
}

std::ostream& operator<<(std::ostream& stream,
                         const HolonomicWaypoint& waypoint) {
  return stream;
  // stream << std::boolalpha;
  // return stream << "{\"x\": " << waypoint.x
  //         << ", \"y\": " << waypoint.y
  //         << ", \"heading\": " << waypoint.heading
  //         << ", \"velocity_x\": " << waypoint.velocityX
  //         << ", \"velocity_y\": " << waypoint.velocityY
  //         << ", \"angular_velocity\": " << waypoint.angularVelocity
  //         << ", \"x_constrained\": " << waypoint.xConstrained
  //         << ", \"y_constrained\": " << waypoint.yConstrained
  //         << ", \"heading_constrained\": " << waypoint.headingConstrained
  //         << ", \"velocity_x_constrained\": " <<
  //         waypoint.velocityXConstrained
  //         << ", \"velocity_y_constrained\": " <<
  //         waypoint.velocityYConstrained
  //         << ", \"velocity_magnitude_constrained\": " <<
  //         waypoint.velocityMagnitudeConstrained
  //         << ", \"angular_velocity_constrained\": " <<
  //         waypoint.angularVelocityConstrained
  //         << ", \"control_interval_count\": " <<
  //         waypoint.controlIntervalCount
  //         << ", \"initial_guess_points\": " << waypoint.initialGuessPoints
  //         << ", \"obstacles\": " << waypoint.obstacles
  //         << "}";
}
}  // namespace trajopt
