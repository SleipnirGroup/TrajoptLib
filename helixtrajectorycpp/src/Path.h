#pragma once

#include <vector>

#include "Waypoint.h"

namespace helixtrajectory {
    class Path {
    public:
        /**
         * @brief Destroy the Path object
         */
        virtual ~Path();
        /**
         * @brief Gets the number of waypoints that make up this path.
         * 
         * @return the length of this path
         */
        virtual size_t Length() const noexcept = 0;
        /**
         * @brief Get the waypoint at the specified index.
         * 
         * @param index the index
         * @return a reference to the waypoint
         */
        virtual Waypoint& GetWaypoint(size_t index) = 0;
        /**
         * @brief Get the waypoint at the specified index.
         * 
         * @param index the index
         * @return a const reference to the waypoint
         */
        virtual const Waypoint& GetWaypoint(size_t index) const = 0;
        /**
         * @brief Get the number of control intervals of the entire path.
         * 
         * @return the total number of control intervals in the path
         */
        size_t ControlIntervalTotal() const;
        /**
         * @brief Checks if this path is valid. A path is valid if it contains
         * no waypoints or the first waypoint has zero control intervals and
         * each waypoint in it is valid.
         * 
         * @return true if this path is valid
         */
        bool IsValid() const noexcept;
    };
}