#include "constraint/VelocityHolonomicConstraint.h"

#include "constraint/Constraint.h"
#include "set/Set2d.h"

namespace helixtrajectory {

VelocityHolonomicConstraint::VelocityHolonomicConstraint(const Set2d& velocityBound, CoordinateSystem coordinateSystem)
        : velocityBound(velocityBound), coordinateSystem(coordinateSystem) {
}
}