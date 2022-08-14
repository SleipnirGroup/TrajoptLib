#include "ObstaclePoint.h"

#include <iostream>

namespace helixtrajectory {

    std::ostream& operator<<(std::ostream& stream, const ObstaclePoint& point) {
        return stream << "{\"x\": " << point.x
                << ", \"y\": " << point.y
                << "}";
    }
}