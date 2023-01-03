#pragma once

#include <optional>

#include "set/IntervalSet1d.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

class RectangularSet2d {
public:
    IntervalSet1d xBound;
    IntervalSet1d yBound;

    RectangularSet2d(const IntervalSet1d& xBound, const IntervalSet1d& yBound);
    static RectangularSet2d PolarExactSet2d(double r, double theta);
    static RectangularSet2d R2();

    std::optional<SolutionError> CheckVector(double xComp, double yComp, const SolutionTolerances& tolerances) const noexcept;

    /**
     * @brief Check if this planar bound is valid. A planar bound is valid when the bounds
     * on a0 and a1 are valid, and additionally for planar bounds, a0 is contained within
     * the interval [0, inf] and a1 is contained within
     * the interval [-pi, pi].
     * 
     * @return true if and only if this planar bound is valid
     */
    bool IsValid() const noexcept;
};
}

template<>
struct fmt::formatter<helixtrajectory::RectangularSet2d> {

    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx) {
        return ctx.begin();
    }

    template<typename FormatContext>
    auto format(const helixtrajectory::RectangularSet2d& rectangularSet, FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "x {}, y {}", rectangularSet.xBound, rectangularSet.yBound);
    }
};