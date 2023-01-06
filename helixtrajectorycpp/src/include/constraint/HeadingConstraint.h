#pragma once

#include <optional>

#include <fmt/format.h>

#include "set/IntervalSet1d.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

class HeadingConstraint {
public:
    IntervalSet1d headingBound;

    HeadingConstraint(const IntervalSet1d& headingBound);

    std::optional<SolutionError> CheckHeading(double theta,
            const SolutionTolerances& tolerances) const noexcept;
};
}

template<>
struct fmt::formatter<helixtrajectory::HeadingConstraint> {

    constexpr auto parse(fmt::format_parse_context& ctx) {
        return ctx.begin();
    }

    template<typename FormatContext>
    auto format(const helixtrajectory::HeadingConstraint& headingConstraint, FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "heading {}", headingConstraint.headingBound);
    }
};