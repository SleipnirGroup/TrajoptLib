#include "HolonomicVectorConstraintSet.h"

namespace helixtrajectory {

    HolonomicVectorConstraintSet::HolonomicVectorConstraintSet(const VectorBound& vectorBound,
            const ScalarBound& rotationBound, bool robotRelative)
            : vectorBound(std::make_unique(vectorBound)), rotationBound(rotationBound), robotRelative(robotRelative) {
    }

    bool HolonomicVectorConstraintSet::IsValid() const noexcept {
        return vectorBound->IsValid() && rotationBound.IsValid();
    }
}