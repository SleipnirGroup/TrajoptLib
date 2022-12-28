#include "set/ConeSet2d.h"

#include <cmath>
#include <optional>

#include <fmt/format.h>

#include "solution/SolutionChecking.h"

namespace helixtrajectory {

ConeSet2d::ConeSet2d(const IntervalSet1d& thetaBound)
        : thetaBound(thetaBound) {
}

std::optional<SolutionError> ConeSet2d::CheckVector(double xComp, double yComp, const SolutionTolerances& tolerances) const noexcept {
    if (!(xComp * std::sin(thetaBound.upper) >= yComp * std::cos(thetaBound.upper) &&
          xComp * std::sin(thetaBound.lower) <= yComp * std::cos(thetaBound.lower))) {
        return SolutionError(fmt::format(
                "({}, {}) is not inside a cone with a theta bound of [{} rad, {} rad]",
                xComp, yComp, thetaBound.lower, thetaBound.upper));
    }
    return std::nullopt;
}

bool ConeSet2d::IsValid() const noexcept {
    return thetaBound.Range() > 0.0 && thetaBound.Range() <= M_PI;
}
}