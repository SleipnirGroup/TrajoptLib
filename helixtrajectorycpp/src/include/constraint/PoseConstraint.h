#pragma once

#include <optional>

#include <fmt/format.h>

#include "constraint/HeadingConstraint.h"
#include "constraint/TranslationConstraint.h"
#include "set/IntervalSet1d.h"
#include "set/Set2d.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

class PoseConstraint : public TranslationConstraint, public HeadingConstraint {
public:
    PoseConstraint(const Set2d& translationBound,
            const IntervalSet1d& headingBound);

    std::optional<SolutionError> CheckPose(double x, double y, double heading,
            const SolutionTolerances& tolerances) const noexcept;
};
}

template<>
struct fmt::formatter<helixtrajectory::PoseConstraint> {

    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx) {
        return ctx.begin();
    }

    template<typename FormatContext>
    auto format(const helixtrajectory::PoseConstraint& poseConstraint,
            FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "obstacle constraint");
    }
};