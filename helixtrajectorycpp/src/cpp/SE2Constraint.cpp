#include "SE2Constraint.h"

namespace helixtrajectory {

    HolonomicVectorConstraintSet::HolonomicVectorConstraintSet(const PlanarBound& translationBound,
            const ScalarBound& rotationBound, CoordinateSystem coordinateSystem)
            : translationBound(translationBound), rotationBound(rotationBound), coordinateSystem(coordinateSystem) {
    }

    bool HolonomicVectorConstraintSet::IsValid() const noexcept {
        return translationBound.IsValid() && rotationBound.IsValid();
    }
}