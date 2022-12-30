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
        double rComp = std::hypot(xComp, yComp);
        double thetaComp = std::atan2(yComp, xComp);
        return SolutionError(fmt::format("(r, θ) = ({}, {})", rComp, thetaComp));
    }
    return std::nullopt;
}

bool ConeSet2d::IsValid() const noexcept {
    return thetaBound.Range() > 0.0 && thetaBound.Range() <= M_PI;
}
}

template<typename ParseContext>
constexpr auto fmt::formatter<helixtrajectory::ConeSet2d>::parse(
        ParseContext& ctx) {
    return ctx.begin();
}

template<typename FormatContext>
auto fmt::formatter<helixtrajectory::ConeSet2d>::format(
        const helixtrajectory::ConeSet2d& coneSet,
        FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "cone: θ = {}", coneSet.thetaBound);
}