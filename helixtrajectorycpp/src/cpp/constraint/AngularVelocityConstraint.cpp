#include "constraint/AngularVelocityConstraint.h"

#include <fmt/format.h>

#include "IncompatibleTrajectoryException.h"
#include "set/IntervalSet1d.h"

namespace helixtrajectory {

AngularVelocityConstraint::AngularVelocityConstraint(const IntervalSet1d& angularVelocityBound)
        : angularVelocityBound(angularVelocityBound) {
}

void AngularVelocityConstraint::CheckAngularVelocity(double angularVelocity) const {
    try {
        angularVelocityBound.CheckScalar(angularVelocity);
    } catch (const IncompatibleTrajectoryException& exception) {
        throw IncompatibleTrajectoryException(fmt::format("angular velocity of {}", exception.what()));
    }
}
}