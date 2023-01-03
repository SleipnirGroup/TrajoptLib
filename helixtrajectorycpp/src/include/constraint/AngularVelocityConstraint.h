#pragma once

#include <optional>

#include <fmt/format.h>

#include "set/IntervalSet1d.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

class AngularVelocityConstraint {
public:
    IntervalSet1d angularVelocityBound;

    AngularVelocityConstraint(const IntervalSet1d& angularVelocityBound);

    std::optional<SolutionError> CheckAngularVelocity(double angularVelocity,
            const SolutionTolerances& tolerances) const noexcept;
};
}

template<>
struct fmt::formatter<helixtrajectory::AngularVelocityConstraint> {

    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx) {
        return ctx.begin();
    }

    template<typename FormatContext>
    auto format(const helixtrajectory::AngularVelocityConstraint& angularVelocityConstraint,
            FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "ω {}",
                angularVelocityConstraint.angularVelocityBound);
    }
};