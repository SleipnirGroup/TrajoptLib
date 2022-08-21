#pragma once

#include <iostream>
#include <vector>

#include "ObstaclePoint.h"

namespace helixtrajectory {

    /**
     * @brief Represents a physical obstacle that the robot must avoid by a certain distance.
     * Arbitrary polygons can be expressed with this class, and circle obstacles can also be
     * created by only using one point with a safety distance. Obstacle points must be wound either
     * clockwise or counterclockwise.
     */
    class Obstacle {
    public:
        /**
         * @brief minimum distance from the obstacle the robot must maintain
         */
        double safetyDistance;
        /**
         * @brief the list of points that make up this obstacle
         */
        std::vector<ObstaclePoint> points;

        /**
         * @brief Construct a new Obstacle object
         * 
         * @param safetyDistance the list of points that make up this obstacle
         * @param waypoints minimum distance from the obstacle the robot must maintain
         */
        Obstacle(double safetyDistance, const std::vector<ObstaclePoint>& points);

        friend std::ostream& operator<<(std::ostream& stream, const Obstacle& obstacle);
    };
}