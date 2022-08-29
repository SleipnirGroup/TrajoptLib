#include "Waypoint.h"

#include <vector>

#include "InitialGuessPoint.h"
#include "Obstacle.h"

namespace helixtrajectory {

    Waypoint::Waypoint(double x, double y, double heading,
            bool xConstrained, bool yConstrained, bool headingConstrained,
            size_t controlIntervalCount,
            const std::vector<InitialGuessPoint>& initialGuessPoints,
            const std::vector<Obstacle>& obstacles)
            : x(x), y(y), heading(heading),
            xConstrained(xConstrained), yConstrained(yConstrained), headingConstrained(headingConstrained),
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