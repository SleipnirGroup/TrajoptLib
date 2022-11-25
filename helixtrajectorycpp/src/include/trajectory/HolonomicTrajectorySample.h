#pragma once

#include <iostream>

#include "trajectory/HolonomicState.h"

namespace helixtrajectory {

class HolonomicTrajectorySample {
public:
    double intervalDuration;
    HolonomicState state;

    HolonomicTrajectorySample(double intervalDuration, const HolonomicState& state);

    void CheckKinematics(const HolonomicState& previousState) const;

    friend std::ostream& operator<<(std::ostream& stream, const HolonomicTrajectorySample& sample);
};
}