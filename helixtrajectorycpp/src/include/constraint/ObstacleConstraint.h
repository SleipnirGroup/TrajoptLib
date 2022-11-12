#pragma once

#include "obstacle/Obstacle.h"

namespace helixtrajectory {

    class ObstacleConstraint {
    public:
        Obstacle obstacle;

        ObstacleConstraint(const Obstacle& obstacle);
    };
}