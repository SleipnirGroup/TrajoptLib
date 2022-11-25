#include "set/EllipticalSet2d.h"

#include <cmath>
#include <limits>

#include <fmt/format.h>

#include "IncompatibleTrajectoryException.h"
#include "TestUtil.h"

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

void EllipticalSet2d::CheckVector(double x, double y) const {
    auto scaledVectorXSquared = (x * x) / (xRadius * xRadius);
    auto scaledVectorYSquared = (y * y) / (yRadius * yRadius);
    auto lhs = scaledVectorXSquared + scaledVectorYSquared;
    switch (direction) {
            case Direction::kInside:
                if (!(lhs <= 1.0)) {
                    throw IncompatibleTrajectoryException(
                            fmt::format("({}, {}) is not on or inside an ellipse with x radius of {} and y radius of {}",
                            x, y, xRadius, yRadius));
                }
                break;
            case Direction::kCentered:
                if (!WithinPrecision(lhs, 1.0, 1e-3)) {
                    throw IncompatibleTrajectoryException(
                            fmt::format("({}, {}) is not on an ellipse with x radius of {} and y radius of {}",
                            x, y, xRadius, yRadius));
                }
                break;
            case Direction::kOutside:
                if (!(lhs >= 1.0)) {
                    throw IncompatibleTrajectoryException(
                            fmt::format("({}, {}) is not on or outside an ellipse with x radius of {} and y radius of {}",
                            x, y, xRadius, yRadius));
                }
                break;
        }
}

bool EllipticalSet2d::IsValid() const noexcept {
    return xRadius > 0.0 && yRadius > 0.0;
}
}