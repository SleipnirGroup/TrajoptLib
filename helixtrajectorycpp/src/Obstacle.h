#pragma once

#include <vector>

namespace helixtrajectory {
    struct ObstaclePoint {
        /**
         * @brief x coordinate of obstacle point
         */
        double x;
        /**
         * @brief y coordinate of obstacle point
         */
        double y;
    };

    /**
     * @brief Represents a physical obstacle that the robot must avoid by a certain distance.
     * Arbitrary polygons can be expressed with this class, and circle obstacles can also be
     * created by only using one point.
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
        Obstacle(double safetyDistance, const std::vector<ObstaclePoint>& waypoints);
    };
}