#pragma once

#include "Path.h"

namespace helixtrajectory {

    /**
     * @brief A waypoint in a holonomic path. This struct includes additional velocity constraints
     * specific to holonomic drivetrains.
     */
    struct HolonomicWaypoint : public Waypoint {
        /**
         * @brief x-coordinate of robot velocity at waypoint
         */
        double vx;
        /**
         * @brief y-coordinate of robot velocity at waypoint
         */
        double vy;
        /**
         * @brief angular velocity of robot at waypoint
         */
        double omega;
        /**
         * @brief whether or not the optimizer should constrain the x-component of velocity of the robot at waypoint
         */
        bool vxConstrained;
        /**
         * @brief whether or not the optimizer should constrain the y-component of velocity of the robot at waypoint
         */
        bool vyConstrained;
        /**
         * @brief whether or not the optimizer should constrain the magnitude of the velocity vector of the robot at waypoint
         */
        bool vMagnitudeConstrained;
        /**
         * @brief whether or not the optimizer should constrain the angular velocity of the robot at waypoint
         */
        bool omegaConstrained;
    };

    /**
     * @brief A sequence of holonomic waypoints that make up a path that the robot can follow. Note that,
     * unlike a path, which is only a loose idea of where the robot should go, a Trajectory is the
     * detailed output of the optimizer that tells the robot exactly how to move.
     */
    class HolonomicPath : public Path {
    public:
        /**
         * @brief the waypoints that make up this path
         */
        std::vector<HolonomicWaypoint> waypoints;
        /**
         * @brief Construct a HolonomicPath with a list of holonomic waypoints
         * 
         * @param waypoints the holonomic waypoints that make up this path
         */
        HolonomicPath(const std::vector<HolonomicWaypoint>& waypoints);

        inline virtual size_t Length() const noexcept;
        inline virtual Waypoint& GetWaypoint(size_t index);
        inline virtual const Waypoint& GetWaypoint(size_t index) const;
    };
}