// Copyright (c) TrajoptLib contributors

#include "path/HolonomicPath.h"

#include <vector>

#include "path/HolonomicWaypoint.h"
#include "path/Path.h"
#include "path/Waypoint.h"

namespace trajopt {

HolonomicPath::HolonomicPath(
    std::vector<HolonomicWaypoint> holonomicWaypoints, const Obstacle& bumpers,
    std::vector<Constraint> globalConstraints,
    std::vector<HolonomicConstraint> globalHolonomicConstraints)
    : Path(bumpers, std::move(globalConstraints)),
      holonomicWaypoints(std::move(holonomicWaypoints)),
      globalHolonomicConstraints(std::move(globalHolonomicConstraints)) {}

size_t HolonomicPath::Length() const noexcept {
  return holonomicWaypoints.size();
}
Waypoint& HolonomicPath::GetWaypoint(size_t index) {
  return holonomicWaypoints[index];
}
const Waypoint& HolonomicPath::GetWaypoint(size_t index) const {
  return holonomicWaypoints[index];
}

}  // namespace trajopt
