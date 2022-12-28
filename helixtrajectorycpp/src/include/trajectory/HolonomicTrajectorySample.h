#pragma once

#include <iostream>
#include <fmt/format.h>

namespace helixtrajectory {

class HolonomicTrajectorySample {
public:
    double timestamp;
    double x;
    double y;
    double heading;
    double velocityX;
    double velocityY;
    double angularVelocity;

    HolonomicTrajectorySample(
            double timestamp,
            double x,
            double y,
            double heading,
            double velocityX,
            double velocityY,
            double angularVelocity);

    friend std::ostream& operator<<(std::ostream& stream, const HolonomicTrajectorySample& sample);
};
}

// template<>
// struct fmt::formatter<HolonomicTrajectorySample> {
//     template<typename ParseContext>
//     constexpr auto parse(ParseContext& ctx);

//     template<typename FormatContext>
//     auto format(const HolonomicTrajectorySample& sample, FormatContext& ctx);
// };