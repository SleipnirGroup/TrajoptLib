#include "constraint/TranslationConstraint.h"

#include "set/Set2d.h"

namespace helixtrajectory {

TranslationConstraint::TranslationConstraint(const Set2d& translationBound)
        : translationBound(translationBound) {
}
}