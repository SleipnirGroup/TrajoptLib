#include "obstacle/ObstaclePoint.h"

#include <iostream>

namespace helixtrajectory {

    ObstaclePoint::ObstaclePoint(double x, double y) : x(x), y(y) {
    }

    std::ostream& operator<<(std::ostream& stream, const ObstaclePoint& point) {
        return stream << "{\"x\": " << point.x
                << ", \"y\": " << point.y
                << "}";
    }
}