#include "trajectory/HolonomicTrajectorySample.h"

#include <iostream>

#include "trajectory/HolonomicState.h"

namespace helixtrajectory {

HolonomicTrajectorySample::HolonomicTrajectorySample(double intervalDuration, const HolonomicState& state)
        : intervalDuration(intervalDuration), state(state) {
}

std::ostream& operator<<(std::ostream& stream, const HolonomicTrajectorySample& sample) {
    return stream << "{\"interval_duration\": " << sample.intervalDuration
            << ", \"state\": " << sample.state
            << "}";
}
}