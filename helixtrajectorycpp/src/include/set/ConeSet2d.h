#pragma once

#include <optional>

#include "set/IntervalSet1d.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

class ConeSet2d {
public:
    IntervalSet1d thetaBound;

    ConeSet2d(const IntervalSet1d& thetaBound);

    std::optional<SolutionError> CheckVector(double xComp, double yComp, const SolutionTolerances& tolerances) const noexcept;

    bool IsValid() const noexcept;
};
}

template<>
struct fmt::formatter<helixtrajectory::ConeSet2d> {

    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx);

    template<typename FormatContext>
    auto format(const helixtrajectory::ConeSet2d& coneSet, FormatContext& ctx);
};