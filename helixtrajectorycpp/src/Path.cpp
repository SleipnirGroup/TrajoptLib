#include "Path.h"

#include <vector>

namespace helixtrajectory {

    Waypoint::Waypoint(double x, double y, double heading,
            bool xConstrained, bool yConstrained, bool headingConstrained,
            const std::vector<InitialGuessPoint>& initialGuessPoints)
            : x(x), y(y), heading(heading), xConstrained(xConstrained), yConstrained(yConstrained), headingConstrained(headingConstrained), initialGuessPoints(initialGuessPoints) {
    }

    Waypoint::~Waypoint() {
    }

    bool Waypoint::IsValid() const noexcept {
        return xConstrained || yConstrained;
    }
    bool Waypoint::IsPositionStateKnown() const noexcept {
        return xConstrained && yConstrained && headingConstrained;
    }
    bool Waypoint::IsStateKnown() const noexcept {
        return IsPositionStateKnown() && IsVelocityStateKnown();
    }

    Path::~Path() {
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