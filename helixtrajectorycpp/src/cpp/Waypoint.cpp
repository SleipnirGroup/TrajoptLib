#include "path/Waypoint.h"

#include <vector>

#include "constraint/PositionConstraint.h"
#include "path/InitialGuessPoint.h"
#include "obstacle/Obstacle.h"

namespace helixtrajectory {

    Waypoint::Waypoint(const PositionConstraint& waypointPositionConstraint,
                const PositionConstraint& segmentPositionConstraint,
                size_t controlIntervalCount,
                const std::vector<InitialGuessPoint>& initialGuessPoints)
            : waypointPositionConstraint(waypointPositionConstraint),
            segmentPositionConstraint(segmentPositionConstraint),
            controlIntervalCount(controlIntervalCount),
            initialGuessPoints(initialGuessPoints) {
    }

    bool Waypoint::IsInitialWaypoint() const noexcept {
        return controlIntervalCount == 0;
    }
    bool Waypoint::IsValid() const noexcept {
        return (IsInitialWaypoint() && initialGuessPoints.size() == 0)
                || (!IsInitialWaypoint() && initialGuessPoints.size() < controlIntervalCount);
    }
    bool Waypoint::IsPositionStateKnown() const noexcept {
        return waypointPositionConstraint.headingBound.IsExact()
                && waypointPositionConstraint.fieldRelativePositionBound.IsExact();
    }
    bool Waypoint::IsStateKnown() const noexcept {
        return IsPositionStateKnown() && IsVelocityStateKnown();
    }
    bool Waypoint::IsSplitWaypoint() const noexcept {
        return IsInitialWaypoint() || IsStateKnown();
    }
}