#include "set/EllipticalSet2d.h"

#include <limits>

namespace helixtrajectory {

EllipticalSet2d::EllipticalSet2d(double xRadius, double yRadius, Direction direction)
        : xRadius(xRadius), yRadius(yRadius), direction(direction) {
}

EllipticalSet2d EllipticalSet2d::CircularSet2d(double radius, Direction direction) {
    return EllipticalSet2d(radius, radius, direction);
}

bool EllipticalSet2d::IsCircular() const noexcept {
    return xRadius == yRadius;
}

bool EllipticalSet2d::IsR2() const noexcept {
    return xRadius >= std::numeric_limits<double>::infinity()
        && yRadius >= std::numeric_limits<double>::infinity();
}

bool EllipticalSet2d::IsValid() const noexcept {
    return xRadius > 0.0 && yRadius > 0.0;
}
}