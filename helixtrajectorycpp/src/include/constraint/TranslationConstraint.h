#pragma once

#include <vector>

#include "obstacle/Obstacle.h"
#include "constraint/PlanarBound.h"
#include "constraint/ScalarBound.h"

namespace helixtrajectory {

    class PositionConstraint {
    public:
        PlanarBound translationBound;

        TranslationConstraint(const PlanarBound& fieldRelativePositionBound);
    };
}