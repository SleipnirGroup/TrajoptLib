#include "HolonomicPath.h"

#include <iostream>
#include <vector>

#include <casadi/casadi.hpp>

#include "HolonomicWaypoint.h"
#include "Path.h"
#include "Waypoint.h"

namespace helixtrajectory {

    HolonomicPath::HolonomicPath(const std::vector<HolonomicWaypoint>& holonomicWaypoints)
        : holonomicWaypoints(holonomicWaypoints) {
    }

    size_t HolonomicPath::Length() const noexcept {
        return holonomicWaypoints.size();
    }
    Waypoint& HolonomicPath::GetWaypoint(size_t index) {
        return holonomicWaypoints[index];
    }
    const Waypoint& HolonomicPath::GetWaypoint(size_t index) const {
        return holonomicWaypoints[index];
    }

    std::ostream& operator<<(std::ostream& stream, const HolonomicPath& path) {
        return stream << path.holonomicWaypoints;
    }
}