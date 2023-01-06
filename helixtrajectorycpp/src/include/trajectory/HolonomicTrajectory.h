#pragma once

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
    constexpr auto parse(ParseContext& ctx) {
        return ctx.begin();
    }

    template<typename FormatContext>
    auto format(const helixtrajectory::HolonomicTrajectory& trajectory, FormatContext& ctx) {
        std::string sampsStr = fmt::format("{}", trajectory.samples[0]);
        for (int i = 1; i < trajectory.samples.size(); i++) {
            sampsStr += fmt::format(", {}", trajectory.samples[i]);
        }
        return fmt::format_to(ctx.out(), "[{}]", sampsStr);
    }
};