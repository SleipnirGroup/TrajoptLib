#include "constraint/HeadingConstraint.h"

#include "set/IntervalSet1d.h"

namespace helixtrajectory {

HeadingConstraint::HeadingConstraint(const IntervalSet1d& headingBound)
        : headingBound(headingBound) {
}
}