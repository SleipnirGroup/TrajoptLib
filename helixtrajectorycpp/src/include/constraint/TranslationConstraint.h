#pragma once

#include "set/Set2d.h"

namespace helixtrajectory {

    class TranslationConstraint {
    public:
        Set2d translationBound;

        TranslationConstraint(const Set2d& translationBound);
    };
}