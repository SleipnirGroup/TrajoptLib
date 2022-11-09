#include "set/EllipticalSet2d.h"

#include <limits>

namespace helixtrajectory {

EllipticalSet2d::EllipticalSet2d(double xRadius, double yRadius, bool invert)
        : xRadius(xRadius), yRadius(yRadius), invert(invert) {
}

EllipticalSet2d::CircularSet2d(double radius, bool invert) {
    return EllipticalSet2d(radius, radius, invert);
}

EllipticalSet2d::IsCircular() const noexcept {
    return xRadius == yRadius;
}

EllipticalSet2d::IsR2() const noexcept {
    return xRadius >= std::numeric_limits<double>::infinity()
        && yRadius >= std::numeric_limits<double>::infinity();
}

EllipticalSet2d::IsValid() const noexcept {
    return xRadius > 0.0 && yRadius > 0.0;
}
}