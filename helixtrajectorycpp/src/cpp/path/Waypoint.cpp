#include "path/Waypoint.h"

#include <vector>

#include "constraint/Constraint.h"
#include "path/InitialGuessPoint.h"
#include "obstacle/Obstacle.h"

namespace helixtrajectory {

    Waypoint::Waypoint(const std::vector<Constraint>& waypointConstraints,
            const std::vector<Constraint>& segmentConstraints,
            size_t controlIntervalCount,
            const std::vector<InitialGuessPoint>& initialGuessPoints)
            : waypointConstraints(waypointConstraints),
            segmentConstraints(segmentConstraints),
            controlIntervalCount(controlIntervalCount),
            initialGuessPoints(initialGuessPoints) {
    }

    bool Waypoint::IsInitialWaypoint() const noexcept {
        return controlIntervalCount == 0;
    }
    bool Waypoint::IsValid() const noexcept {
        return initialGuessPoints.size() <= controlIntervalCount;
    }
    bool Waypoint::IsPositionStateKnown() const noexcept {
        // return waypointPositionConstraint.headingBound.IsExact()
        //         && waypointPositionConstraint.fieldRelativePositionBound.IsExact();
        return false;
    }
    bool Waypoint::IsStateKnown() const noexcept {
        return IsPositionStateKnown() && IsVelocityStateKnown();
    }
    bool Waypoint::IsSplitWaypoint() const noexcept {
        return IsInitialWaypoint() || IsStateKnown();
    }
}