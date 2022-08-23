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

    Waypoint::~Waypoint() {
    }

    bool Waypoint::IsValid() const noexcept {
        return xConstrained || yConstrained &&
                initialGuessPoints.size() < controlIntervalCount;
                // ^note this also checks that there is at least one control interval
    }
    bool Waypoint::IsPositionStateKnown() const noexcept {
        return xConstrained && yConstrained && headingConstrained;
    }
    bool Waypoint::IsStateKnown() const noexcept {
        return IsPositionStateKnown() && IsVelocityStateKnown();
    }
}