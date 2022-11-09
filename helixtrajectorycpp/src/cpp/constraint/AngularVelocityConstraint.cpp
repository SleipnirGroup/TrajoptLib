#include "constraint/AngularVelocityConstraint.h"

#include "set/IntervalSet2d.h"

namespace helixtrajectory {

AngularVelocityConstraint::AngularVelocityConstraint(const IntervalSet2d& angularVelocityBound)
        : angularVelocityBound(angularVelocityBound) {
}
}