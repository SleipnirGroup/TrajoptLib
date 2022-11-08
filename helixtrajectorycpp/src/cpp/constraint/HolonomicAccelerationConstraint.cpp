#include "constraint/HolonomicAccelerationConstraint.h"

#include "constraint/PlanarBound.h"
#include "constraint/ScalarBound.h"

namespace helixtrajectory {

    HolonomicAccelerationConstraint::HolonomicAccelerationConstraint(
                const ScalarBound& angularAccelerationBound,
                const PlanarBound& fieldRelativeAccelerationBound,
                const PlanarBound& robotRelativeAccelerationBound)
        : angularAccelerationBound(angularAccelerationBound),
        fieldRelativeAccelerationBound(fieldRelativeAccelerationBound),
        robotRelativeAccelerationBound(robotRelativeAccelerationBound) {
    }
}