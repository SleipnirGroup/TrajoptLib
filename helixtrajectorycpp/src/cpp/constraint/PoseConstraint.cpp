#include "constraint/PoseConstraint.h"

#include "constraint/HeadingConstraint.h"
#include "constraint/TranslationConstraint.h"
#include "set/IntervalSet1d.h"
#include "set/Set2d.h"

namespace helixtrajectory {

PoseConstraint::PoseConstraint(const Set2d& translationBound,
        const IntervalSet1d& headingBound)
        : TranslationConstraint(translationBound), HeadingConstraint(headingBound) {
}
}