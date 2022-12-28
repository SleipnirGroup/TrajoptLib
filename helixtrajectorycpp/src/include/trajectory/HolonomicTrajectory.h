#pragma once

#include <iostream>
#include <vector>

#include "trajectory/HolonomicTrajectorySample.h"

namespace helixtrajectory {

class HolonomicTrajectory {
public:
    std::vector<HolonomicTrajectorySample> samples;

    HolonomicTrajectory(const std::vector<HolonomicTrajectorySample>& samples);
    HolonomicTrajectory(std::vector<HolonomicTrajectorySample>&& samples);

    /**
     * @brief Append a string representation of a holonomic trajectory to
     * an output stream. A string representation of a holonomic trajectory
     * is a json array of its trajectory segments' json representations.
     * 
     * @param stream the stream to append the string representation to
     * @param trajectory the holonomic trajectory
     * @return a reference to the given stream
     */
    // friend std::ostream& operator<<(std::ostream& stream, const HolonomicTrajectory& trajectory);
};
}

// template<>
// struct fmt::formatter<HolonomicTrajectorySample> : fmt::formatter<std::vector>> {
//     template<typename ParseContext>
//     constexpr auto parse(ParseContext& ctx);

//     template<typename FormatContext>
//     auto format(const HolonomicTrajectorySample& sample, FormatContext& ctx);
// };