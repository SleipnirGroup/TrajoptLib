#pragma once

#include <iostream>

namespace helixtrajectory {

    /**
     * @brief A point of an obstacle, usually representing a vertex of a polygon.
     */
    struct ObstaclePoint {
        /**
         * @brief the x-coordinate of obstacle point
         */
        double x;
        /**
         * @brief the y-coordinate of obstacle point
         */
        double y;

        friend std::ostream& operator<<(std::ostream& stream, const ObstaclePoint& point);
    };
}