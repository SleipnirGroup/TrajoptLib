#pragma once

#include <iostream>

#include "trajectory/HolonomicState.h"

namespace helixtrajectory {

class HolonomicTrajectorySample {
public:
    double intervalDuration;
    HolonomicState state;

    HolonomicTrajectorySample(double intervalDuration, const HolonomicState& state);

    friend std::ostream& operator<<(std::ostream& stream, const HolonomicTrajectorySample& sample);
};
}