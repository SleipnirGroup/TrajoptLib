#pragma once

#include <iostream>
#include <vector>

#include "HolonomicWaypoint.h"
#include "Path.h"

namespace helixtrajectory {

    /**
     * @brief A sequence of holonomic waypoints that make up a path that a holonomic drivetrain robot
     * can follow. Note that, unlike a path, which is only a general idea of where the robot should go,
     * a Trajectory is the detailed output of the generator that tells the robot exactly how to move.
     */
    class HolonomicPath : public Path {
    public:
        /**
         * @brief the holonomic waypoints that make up this path
         */
        std::vector<HolonomicWaypoint> holonomicWaypoints;
        /**
         * @brief Construct a HolonomicPath with a list of holonomic waypoints
         * 
         * @param waypoints the holonomic waypoints that make up this path
         */
        HolonomicPath(const std::vector<HolonomicWaypoint>& holonomicWaypoints);

        /**
         * @brief Destroy the Holonomic Path object
         */
        virtual ~HolonomicPath();

        virtual size_t Length() const noexcept;
        virtual Waypoint& GetWaypoint(size_t index);
        virtual const Waypoint& GetWaypoint(size_t index) const;

        friend std::ostream& operator<<(std::ostream& stream, const HolonomicPath& path);
    };
}