#include <trajectory/HolonomicTrajectory.h>

// #include <iostream>

// #include <fmt/format.h>
// #include <fmt/ranges.h>

// #include "IncompatibleTrajectoryException.h"
// #include "trajectory/HolonomicState.h"

namespace helixtrajectory {

HolonomicTrajectory::HolonomicTrajectory(const std::vector<HolonomicTrajectorySample>& samples)
      : samples(samples) {
}

HolonomicTrajectory::HolonomicTrajectory(std::vector<HolonomicTrajectorySample>&& samples)
      : samples(std::move(samples)) {
}
}

template<typename ParseContext>
constexpr auto fmt::formatter<helixtrajectory::HolonomicTrajectory>::parse(ParseContext& ctx) {
    return ctx.begin();
}

template<typename FormatContext>
auto fmt::formatter<helixtrajectory::HolonomicTrajectory>::format(
        const helixtrajectory::HolonomicTrajectory& trajectory, FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "traj1");
}