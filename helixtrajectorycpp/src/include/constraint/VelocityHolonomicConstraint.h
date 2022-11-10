#pragma once

#include "constraint/Constraint.h"
#include "set/Set2d.h"

namespace helixtrajectory {

    class VelocityHolonomicConstraint {
    public:
        Set2d velocityBound;
        CoordinateSystem coordinateSystem;

        VelocityHolonomicConstraint(const Set2d& velocityBound, CoordinateSystem coordinateSystem = CoordinateSystem::kField);
    };
}