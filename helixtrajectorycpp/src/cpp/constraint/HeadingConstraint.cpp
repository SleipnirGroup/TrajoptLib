#include "constraint/HeadingConstraint.h"

#include "set/IntervalSet2d.h"

namespace helixtrajectory {

HeadingConstraint::HeadingConstraint(const IntervalSet2d& headingBound)
        : headingBound(headingBound) {
}
}