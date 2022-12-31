#pragma once

#include <optional>

#include "set/IntervalSet1d.h"
#include "set/RectangularSet2d.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

class LinearSet2d {
public:
    double theta;

    LinearSet2d(double theta);

    std::optional<SolutionError> CheckVector(double xComp, double yComp,
            const SolutionTolerances& tolerances) const noexcept;

    static RectangularSet2d RBoundToRectangular(double theta, const IntervalSet1d& rBound);
};
}

template<>
struct fmt::formatter<helixtrajectory::LinearSet2d> {

    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx);

    template<typename FormatContext>
    auto format(const helixtrajectory::LinearSet2d& linearSet, FormatContext& ctx);
};