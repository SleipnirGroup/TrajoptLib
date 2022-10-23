#include "constraint/HolonomicVelocityConstraint.h"

#include "constraint/PlanarBound.h"
#include "constraint/ScalarBound.h"

namespace helixtrajectory {

    HolonomicVelocityConstraint::HolonomicVelocityConstraint(
                const ScalarBound& angularVelocityBound,
                const PlanarBound& fieldRelativeVelocityBound,
                const PlanarBound& robotRelativeVelocityBound)
        : angularVelocityBound(angularVelocityBound),
        fieldRelativeVelocityBound(fieldRelativeVelocityBound),
        robotRelativeVelocityBound(robotRelativeVelocityBound) {
    }
}