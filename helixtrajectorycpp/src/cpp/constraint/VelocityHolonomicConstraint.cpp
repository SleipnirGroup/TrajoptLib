#include "constraint/VelocityHolonomicConstraint"

#include "constraint/Constraint.h"
#include "set/Set2d.h"

namespace helixtrajectory {
dinateSystem coordinateSystem;

HolonomicVelocityConstraint::HolonomicVelocityConstraint(const Set2d& velocityBound, CoordinateSystem coordinateSystem)
        : velocityBound(velocityBound), coordinateSystem(coordinateSystem) {
}
}