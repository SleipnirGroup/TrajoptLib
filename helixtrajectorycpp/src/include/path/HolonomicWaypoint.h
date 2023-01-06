#pragma once

#include <iostream>
#include <vector>

#include "constraint/HolonomicConstraint.h"
#include "constraint/Constraint.h"
#include "path/InitialGuessPoint.h"
#include "obstacle/Obstacle.h"
#include "path/Waypoint.h"

namespace helixtrajectory {

    /**
     * @brief A waypoint in a holonomic path. This class includes additional velocity constraints
     * specific to holonomic drivetrains.
     * 
     * @author Justin Babilino
     */
    class HolonomicWaypoint : public Waypoint {
    public:
        std::vector<HolonomicConstraint> waypointHolonomicConstraints;
        std::vector<HolonomicConstraint> segmentHolonomicConstraints;

        /**
         * @brief Construct a new Holonomic Waypoint object with its position and velocity state and
         * constraint options, control interval count, and initial guess points.
         * 
         * @param x the x-coordinate of the robot at this waypoint
         * @param y the y-coordinate of the robot at this waypoint
         * @param heading the heading of the robot at this waypoint
         * @param velocityX the x-component of robot velocity at this waypoint
         * @param velocityY the y-component of robot velocity at this waypoint
         * @param angularVelocity the angular velocity of the robot at this waypoint
         * @param xConstrained whether or not the optimizer should constrain the
         *                     x-coordinate of the robot at this waypoint
         * @param yConstrained whether or not the optimizer should constrain the
         *                     y-coordinate of the robot at this waypoint
         * @param headingConstrained whether or not the optimizer should constrain
         *                           the heading of the robot at this waypoint
         * @param velocityXConstrained whether or not the optimizer should constrain
         *                             the x-component of velocity of the robot at this waypoint
         * @param velocityYConstrained whether or not the optimizer should constrain the
         *                             y-component of velocity of the robot at this waypoint
         * @param velocityMagnitudeConstrained whether or not the optimizer should constrain the
         *                                     magnitude of the velocity vector of the robot at this waypoint
         * @param angularVelocityConstrained whether or not the optimizer should constrain
         *                                   the angular velocity of the robot at this waypoint
         * @param controlIntervalCount the number of control intervals in the optimization
         *                             problem from the previous waypoint to this waypoint
         * @param initialGuessPoints the points used to construct the linear initial trajectory
         *                           guess for the trajectory segment from the last waypoint to this waypoint
         * @param obstacles the collection of obstacles that the robot must avoid while approaching this waypoint
         */
        HolonomicWaypoint(
                const std::vector<Constraint>& waypointConstraints,
                const std::vector<HolonomicConstraint>& waypointHolonomicConstraints = {},
                const std::vector<Constraint>& segmentConstraints = {},
                const std::vector<HolonomicConstraint>& segmentHolonomicConstraints = {},
                size_t controlIntervalCount = 100,
                const std::vector<InitialGuessPoint>& initialGuessPoints = {});

        /**
         * @brief Check if the velocity state at this holonomic waypoint is known. This
         * means that the velocity vector and angular velocity of the robot
         * is constrained.
         * 
         * @return true if the velocity state of the robot is constrained, false otherwise
         */
        bool IsVelocityStateKnown() const noexcept override;

        /**
         * @brief Append a string representation of a holonomic waypoint
         * to an output stream. A string representation of a holonomic waypoint
         * is a json object with the same fields as this object.
         * 
         * @param stream the stream to append the string representation to
         * @param waypoint the holonomic waypoint
         * @return a reference to the given stream
         */
        friend std::ostream& operator<<(std::ostream& stream, const HolonomicWaypoint& waypoint);
    };
}

template<>
struct fmt::formatter<helixtrajectory::HolonomicWaypoint> {

    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx) {
        return ctx.begin();
    }

    template<typename FormatContext>
    auto format(const helixtrajectory::HolonomicWaypoint& holonomicWaypoint,
            FormatContext& ctx) {
        return fmt::format_to(ctx.out(),
            "Holonomic Waypoint");
    }
};