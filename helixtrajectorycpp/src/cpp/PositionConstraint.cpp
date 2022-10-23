#include "constraint/PositionConstraint.h"

#include <vector>

#include "constraint/PlanarBound.h"
#include "constraint/ScalarBound.h"
#include "obstacle/Obstacle.h"

namespace helixtrajectory {

    PositionConstraint::PositionConstraint(const ScalarBound& headingBound,
            const PlanarBound& fieldRelativePositionBound,
            const std::vector<Obstacle>& obstacles)
        : headingBound(headingBound), fieldRelativePositionBound(fieldRelativePositionBound),
        obstacles(obstacles) {
    }
}