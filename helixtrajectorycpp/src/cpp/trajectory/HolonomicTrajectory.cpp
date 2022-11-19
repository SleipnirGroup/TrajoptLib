#include <trajectory/HolonomicTrajectory.h>

#include <iostream>

#include <casadi/casadi.hpp>

#include "trajectory/HolonomicState.h"

namespace helixtrajectory {

HolonomicTrajectory::HolonomicTrajectory(const HolonomicState initialState, const std::vector<HolonomicTrajectorySample>& samples)
        : initialState(initialState), samples(samples) {
}

std::ostream& operator<<(std::ostream& stream, const HolonomicTrajectory& trajectory) {
    return stream << "{\"initial_state\": " << sample.initialState
            << ", \"samples\": " << sample.samples
            << "}";
    // stream << "[";
    // double ts = 0.0;
    // for (auto& sample : trajectory.samples) {
    //     ts += sample.intervalDuration;
    //     stream << "{\"ts\": " << ts
    //         << ", \"x\": " << sample.x
    //         << ", \"y\": " << sample.y
    //         << ", \"heading\": " << sample.heading
    //         << ", \"vx\": " << 0.0
    //         << ", \"vy\": " << 0.0
    //         << ", \"omega\": " << 0.0
    //         << "},";
    // }
    // return stream;
}
}