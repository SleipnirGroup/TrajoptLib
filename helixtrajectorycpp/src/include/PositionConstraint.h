#pragma once

#include <vector>

#include "Obstacle.h"
#include "PlanarBound.h"
#include "ScalarBound.h"

namespace helixtrajectory {

    class PositionConstraint {
    public:
        /**
         * @brief the collection of obstacles that the robot must avoid at a sample point
         */
        ScalarBound headingBound;
        PlanarBound fieldRelativePositionBound;
        std::vector<Obstacle> obstacles;

        PositionConstraint(const ScalarBound& headingBound = ScalarBound(),
                const PlanarBound& fieldRelativePositionBound = PlanarBound(),
                const std::vector<Obstacle>& obstacles = {});
    };
}