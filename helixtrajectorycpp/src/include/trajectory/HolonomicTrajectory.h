#pragma once

#include <iostream>
#include <vector>

#include <fmt/format.h>

#include "trajectory/HolonomicTrajectorySample.h"

namespace helixtrajectory {

class HolonomicTrajectory {
public:
    std::vector<HolonomicTrajectorySample> samples;

    HolonomicTrajectory(const std::vector<HolonomicTrajectorySample>& samples);
    HolonomicTrajectory(std::vector<HolonomicTrajectorySample>&& samples);
};
}

template<>
struct fmt::formatter<helixtrajectory::HolonomicTrajectory> {
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx);

    template<typename FormatContext>
    auto format(const helixtrajectory::HolonomicTrajectory& trajectory, FormatContext& ctx);
};