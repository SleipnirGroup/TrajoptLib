#pragma once

#include <iostream>

#include "trajectory/SwerveState.h"

namespace helixtrajectory {

class SwerveTrajectorySample {
public:
    double intervalDuration;
    SwerveState state;

    SwerveTrajectorySample(double intervalDuration, const SwerveState& state);

    void CheckKinematics(const SwerveState& previousState) const;

    friend std::ostream& operator<<(std::ostream& stream, const SwerveTrajectorySample& sample);
};
}