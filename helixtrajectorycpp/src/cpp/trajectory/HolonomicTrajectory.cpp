#include <trajectory/HolonomicTrajectory.h>

#include <iostream>

#include <casadi/casadi.hpp>
#include <fmt/format.h>

#include "IncompatibleTrajectoryException.h"
#include "trajectory/HolonomicState.h"

namespace helixtrajectory {

HolonomicTrajectory::HolonomicTrajectory(const HolonomicState& initialState, const std::vector<HolonomicTrajectorySample>& samples)
        : initialState(initialState), samples(samples) {
}

void HolonomicTrajectory::CheckKinematics() const {
    for (std::size_t sampleIndex = 1; sampleIndex < samples.size() + 1; sampleIndex++) {
        const HolonomicState* previousState = nullptr;
        auto& currentSample = samples[sampleIndex - 1];
        auto intervalDuration = currentSample.intervalDuration;
        if (sampleIndex == 1) {
            previousState = &initialState;
        } else {
            previousState = &samples[sampleIndex - 2].state;
        }
        try {
            currentSample.CheckKinematics(*previousState);
        } catch (const IncompatibleTrajectoryException& exception) {
            throw IncompatibleTrajectoryException(fmt::format("At trajectory sample index {}, {}", sampleIndex, exception.what()));
        }
    }
}

std::ostream& operator<<(std::ostream& stream, const HolonomicTrajectory& trajectory) {
    return stream << "{\"initial_state\": " << trajectory.initialState
            << ", \"samples\": " << trajectory.samples
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