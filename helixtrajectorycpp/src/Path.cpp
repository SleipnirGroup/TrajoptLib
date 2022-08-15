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
        if (Length() == 0 || ControlIntervalTotal() == 0
                || !GetWaypoint(0).IsStateKnown() || !GetWaypoint(Length() - 1).IsStateKnown()) {
            return false;
        }

        for (int i = 1; i < Length(); i++) {
            if (!GetWaypoint(i).IsValid()) {
                return false;
            }
        }
        return true;
    }
}