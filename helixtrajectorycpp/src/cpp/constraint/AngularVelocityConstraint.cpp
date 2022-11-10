#include "constraint/AngularVelocityConstraint.h"

#include "set/IntervalSet1d.h"

namespace helixtrajectory {

AngularVelocityConstraint::AngularVelocityConstraint(const IntervalSet1d& angularVelocityBound)
        : angularVelocityBound(angularVelocityBound) {
}
}