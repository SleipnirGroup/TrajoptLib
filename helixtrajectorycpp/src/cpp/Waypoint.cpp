#include "Waypoint.h"

#include <vector>

#include "InitialGuessPoint.h"
#include "Obstacle.h"

namespace helixtrajectory {

    Waypoint::Waypoint(const HolonomicVectorConstraintSet& waypointPositionConstraintSet,
            const HolonomicVectorConstraintSet& segmentPositionConstraintSet,
            bool applySegmentConstraintsToWaypoint,
            size_t controlIntervalCount,
            const std::vector<InitialGuessPoint>& initialGuessPoints,
            const std::vector<Obstacle>& obstacles)
            : waypointPositionConstraintSet(std::move(waypointPositionConstraintSet)),
            segmentPositionConstraintSet(segmentPositionConstraintSet),
            applySegmentConstraintsToWaypoint(applySegmentConstraintsToWaypoint),
            controlIntervalCount(controlIntervalCount), initialGuessPoints(initialGuessPoints),
            obstacles(obstacles) {
    }

    bool Waypoint::IsInitialWaypoint() const noexcept {
        return controlIntervalCount == 0;
    }
    bool Waypoint::IsValid() const noexcept {
        return (IsInitialWaypoint() && initialGuessPoints.size() == 0)
                || (!IsInitialWaypoint() && initialGuessPoints.size() < controlIntervalCount);
    }
    bool Waypoint::IsPositionStateKnown() const noexcept {
        return xConstrained && yConstrained && headingConstrained;
    }
    bool Waypoint::IsStateKnown() const noexcept {
        return IsPositionStateKnown() && IsVelocityStateKnown();
    }
    bool Waypoint::IsSplitWaypoint() const noexcept {
        return IsInitialWaypoint() || IsStateKnown();
    }
}