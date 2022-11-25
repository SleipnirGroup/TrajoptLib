#include "set/ConeSet2d.h"

#include <cmath>

#include <fmt/format.h>

#include "IncompatibleTrajectoryException.h"

namespace helixtrajectory {

ConeSet2d::ConeSet2d(const IntervalSet1d& thetaBound)
        : thetaBound(thetaBound) {
}

void ConeSet2d::CheckVector(double x, double y) const {
    if (!(x * std::sin(thetaBound.upper) >= y * std::cos(thetaBound.upper) &&
          x * std::sin(thetaBound.lower) <= y * std::cos(thetaBound.lower))) {
        throw IncompatibleTrajectoryException(
                fmt::format("of ({}, {}) is not inside a cone with a theta bound of [{} rad, {} rad]",
                x, y, thetaBound.lower, thetaBound.upper));
    }
}

bool ConeSet2d::IsValid() const noexcept {
    return thetaBound.Range() > 0.0 && thetaBound.Range() <= M_PI;
}
}