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
        inline virtual size_t Length() const noexcept = 0;
        /**
         * @brief Get the waypoint at the specified index.
         * 
         * @param index the index
         * @return a reference to the waypoint
         */
        inline virtual Waypoint& GetWaypoint(size_t index) = 0;
        /**
         * @brief Get the waypoint at the specified index.
         * 
         * @param index the index
         * @return a const reference to the waypoint
         */

        inline virtual const Waypoint& GetWaypoint(size_t index) const = 0;
        bool IsValid() const noexcept;
    };
}