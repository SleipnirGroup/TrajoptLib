#include "HolonomicPath.h"

#include <vector>

#include "Path.h"

namespace helixtrajectory {

    HolonomicWaypoint::HolonomicWaypoint(double x, double y, double heading, double vx, double vy, double omega,
            bool xConstrained, bool yConstrained, bool headingConstrained, bool vxConstrained, bool vyConstrained, bool vMagnitudeConstrained, bool omegaConstrained,
            const std::vector<InitialGuessPoint>& initialGuessPoints)
            : Waypoint(x, y, heading, xConstrained, yConstrained, headingConstrained, initialGuessPoints),
            vx(vx), vy(vy), omega(omega),
            vxConstrained(vxConstrained), vyConstrained(vyConstrained),
            vMagnitudeConstrained(vMagnitudeConstrained), omegaConstrained(omegaConstrained) {
    }

    HolonomicWaypoint::~HolonomicWaypoint() {
    }

    bool HolonomicWaypoint::IsVelocityStateKnown() const noexcept {
        return (vMagnitudeConstrained && vxConstrained && vyConstrained) || 
                ((vx == 0.0 && vy == 0.0) && (
                (!vMagnitudeConstrained && vxConstrained && vyConstrained) ||
                (vMagnitudeConstrained && !vxConstrained && !vyConstrained)));
    }

    HolonomicPath::HolonomicPath(const std::vector<HolonomicWaypoint>& waypoints)
        : waypoints(waypoints) {
    }

    HolonomicPath::~HolonomicPath() {
    }

    inline size_t HolonomicPath::Length() const noexcept {
        return waypoints.size();
    }
    Waypoint& HolonomicPath::GetWaypoint(size_t index) {
        return waypoints[index];
    }
    const Waypoint& HolonomicPath::GetWaypoint(size_t index) const {
        return waypoints[index];
    }
}