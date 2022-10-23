#include "path/HolonomicPath.h"

#include <iostream>
#include <vector>

#include <casadi/casadi.hpp>

#include "path/HolonomicWaypoint.h"
#include "path/Path.h"
#include "path/Waypoint.h"

namespace helixtrajectory {

    HolonomicPath::HolonomicPath(const std::vector<HolonomicWaypoint>& holonomicWaypoints,
            const Obstacle& bumpers,
            const PositionConstraint& globalPositionConstraint,
            const HolonomicVelocityConstraint& globalVelocityConstraint,
            const HolonomicAccelerationConstraint& globalAccelerationConstraint)
            : Path(bumpers, globalPositionConstraint),
            holonomicWaypoints(holonomicWaypoints),
            globalVelocityConstraint(globalVelocityConstraint),
            globalAccelerationConstraint(globalAccelerationConstraint) {
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