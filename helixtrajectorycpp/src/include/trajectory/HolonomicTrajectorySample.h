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

template<>
struct fmt::formatter<helixtrajectory::HolonomicTrajectorySample> {

    constexpr auto parse(fmt::format_parse_context& ctx) {
        return ctx.begin();
    }

    template<typename FormatContext>
    auto format(const helixtrajectory::HolonomicTrajectorySample& sample, FormatContext& ctx) {
        return fmt::format_to(ctx.out(),
            "{{\"timestamp\": {}, \"x\": {}, \"y\": {}, \"heading\": {}, \"velocityX\": {}, \"velocityY\": {}, \"angularVelocity\": {}}}",
                sample.timestamp,
                sample.x, sample.y, sample.heading,
                sample.velocityX, sample.velocityY, sample.angularVelocity);
    }
};