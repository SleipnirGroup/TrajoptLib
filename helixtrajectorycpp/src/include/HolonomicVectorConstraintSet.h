#pragma once

#include <memory>

#include "ScalarBound.h"
#include "VectorBound.h"

namespace helixtrajectory {

    class HolonomicVectorConstraintSet {
    public:
        std::unique_ptr<VectorBound> vectorBound;
        ScalarBound rotationBound;
        bool robotRelative;

        HolonomicVectorConstraintSet(const VectorBound& vectorBound, const ScalarBound& rotationBound, bool robotRelative);

        bool IsValid() const noexcept;
    };
}