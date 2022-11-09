#include "path/Path.h"

#include <vector>

#include "constraint/PositionConstraint.h"
#include "obstacle/Obstacle.h"
#include "path/Waypoint.h"

namespace helixtrajectory {

    Path::Path(const Obstacle& bumpers, const PositionConstraint& globalPositionConstraint)
            : bumpers(bumpers), globalPositionConstraint(globalPositionConstraint) {
    }

    size_t Path::ControlIntervalTotal() const {
        size_t controlIntervalTotal = 0;
        for (size_t waypointIndex = 1; waypointIndex < Length(); waypointIndex++) {
            controlIntervalTotal += GetWaypoint(waypointIndex).controlIntervalCount;
        }
        return controlIntervalTotal;
    }

    bool Path::IsValid() const noexcept {
        if (Length() == 0 || ControlIntervalTotal() == 0
                || !GetWaypoint(0).IsStateKnown() || !GetWaypoint(Length() - 1).IsStateKnown()) {
            return false;
        }
        for (int index = 0; index < Length(); index++) {
            if (!GetWaypoint(index).IsValid()) {
                return false;
            }
        }
        return true;
    }
}