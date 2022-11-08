#pragma once

#include "constraint/PlanarBound.h"
#include "constraint/ScalarBound.h"
#include "constraint/TranslationConstraint.h"

namespace helixtrajectory {

    class PoseConstraint : public TranslationConstraint {
    public:
        ScalarBound headingBound;

        PoseConstraint(const PlanarBound& translationBound,
                const ScalarBound& headingBound);
    };
}