#include "constraint/HeadingConstraint.h"

#include <fmt/format.h>

#include "IncompatibleTrajectoryException.h"
#include "set/IntervalSet1d.h"

namespace helixtrajectory {

HeadingConstraint::HeadingConstraint(const IntervalSet1d& headingBound)
        : headingBound(headingBound) {
}

void HeadingConstraint::CheckHeading(double heading) const {
    try {
        headingBound.CheckScalar(heading);
    } catch (const IncompatibleTrajectoryException& exception) {
        throw IncompatibleTrajectoryException(fmt::format("heading of {}", exception.what()));
    }
}
}