#include "HolonomicPath.h"

#include <vector>

#include "Path.h"

namespace helixtrajectory {

    HolonomicPath::HolonomicPath(const std::vector<HolonomicWaypoint>& waypoints)
        : waypoints(waypoints) {
    }

    inline size_t HolonomicPath::Length() const noexcept {
        return waypoints.size();
    }
    Waypoint& HolonomicPath::GetWaypoint(size_t index) {
        return waypoints[index];
    }
    const Waypoint& HolonomicPath::GetWaypoint(size_t index) const {
        return waypoints[index];
    }
}