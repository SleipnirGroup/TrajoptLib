#include "Path.h"

#include <vector>

namespace helixtrajectory {

    Path::~Path() {
    }

    size_t Path::ControlIntervalTotal() const {
        size_t controlIntervalTotal = 0;
        for (size_t waypointIndex = 1; waypointIndex < Length(); waypointIndex++) {
            controlIntervalTotal += GetWaypoint(waypointIndex).controlIntervalCount;
        }
        return controlIntervalTotal;
    }

    bool Path::IsValid() const noexcept {
        if (Length() == 0) {
            return false;
        }
        if (!GetWaypoint(0).IsStateKnown()) {
            return false;
        }
        for (int i = 1; i < Length() - 1; i++) {
            if (!GetWaypoint(i).IsValid()) {
                return false;
            }
        }
        if (!GetWaypoint(Length() - 1).IsStateKnown()) {
            return false;
        }
        return true;
    }
}