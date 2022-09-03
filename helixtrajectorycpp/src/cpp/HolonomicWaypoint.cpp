#include "HolonomicWaypoint.h"

#include <vector>

#include <casadi/casadi.hpp>

#include "InitialGuessPoint.h"
#include "Obstacle.h"
#include "Waypoint.h"

namespace helixtrajectory {

    HolonomicWaypoint::HolonomicWaypoint(double x, double y, double heading,
            double velocityX, double velocityY, double angularVelocity,
            bool xConstrained, bool yConstrained, bool headingConstrained,
            bool velocityXConstrained, bool velocityYConstrained,
            bool velocityMagnitudeConstrained, bool angularVelocityConstrained,
            size_t controlIntervalCount,
            const std::vector<InitialGuessPoint>& initialGuessPoints,
            const std::vector<Obstacle>& obstacles)
            : Waypoint(x, y, heading, xConstrained, yConstrained, headingConstrained,
            controlIntervalCount, initialGuessPoints, obstacles),
            velocityX(velocityX), velocityY(velocityY), angularVelocity(angularVelocity),
            velocityXConstrained(velocityXConstrained), velocityYConstrained(velocityYConstrained),
            velocityMagnitudeConstrained(velocityMagnitudeConstrained), angularVelocityConstrained(angularVelocityConstrained) {
    }

    bool HolonomicWaypoint::IsVelocityStateKnown() const noexcept {
        return (velocityMagnitudeConstrained && velocityXConstrained && velocityYConstrained)
                || ((velocityX == 0.0 && velocityY == 0.0)
                && ((!velocityMagnitudeConstrained && velocityXConstrained && velocityYConstrained)
                || (velocityMagnitudeConstrained && !velocityXConstrained && !velocityYConstrained)));
    }

    std::ostream& operator<<(std::ostream& stream, const HolonomicWaypoint& waypoint) {
        stream << std::boolalpha;
        return stream << "{\"x\": " << waypoint.x
                << ", \"y\": " << waypoint.y
                << ", \"heading\": " << waypoint.heading
                << ", \"velocity_x\": " << waypoint.velocityX
                << ", \"velocity_y\": " << waypoint.velocityY
                << ", \"angular_velocity\": " << waypoint.angularVelocity
                << ", \"x_constrained\": " << waypoint.xConstrained
                << ", \"y_constrained\": " << waypoint.yConstrained
                << ", \"heading_constrained\": " << waypoint.headingConstrained
                << ", \"velocity_x_constrained\": " << waypoint.velocityXConstrained
                << ", \"velocity_y_constrained\": " << waypoint.velocityYConstrained
                << ", \"velocity_magnitude_constrained\": " << waypoint.velocityMagnitudeConstrained
                << ", \"angular_velocity_constrained\": " << waypoint.angularVelocityConstrained
                << ", \"control_interval_count\": " << waypoint.controlIntervalCount
                << ", \"initial_guess_points\": " << waypoint.initialGuessPoints
                << ", \"obstacles\": " << waypoint.obstacles
                << "}";
    }
}