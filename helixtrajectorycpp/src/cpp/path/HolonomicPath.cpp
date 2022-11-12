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
                const std::vector<Constraint>& globalConstraints,
                const std::vector<HolonomicConstraint>& globalHolonomicConstraints)
            : Path(bumpers, globalConstraints),
            holonomicWaypoints(holonomicWaypoints),
            globalHolonomicConstraints(globalHolonomicConstraints) {
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