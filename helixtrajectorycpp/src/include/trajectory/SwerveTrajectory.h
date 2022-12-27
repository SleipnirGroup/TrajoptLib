#pragma once

#include <iostream>
#include <vector>

#include "trajectory/SwerveState.h"
#include "trajectory/SwerveTrajectorySample.h"

namespace helixtrajectory {

class SwerveTrajectory {
public:
    std::vector<HolonomicTrajectorySample> samples;

    SwerveTrajectory(const std::vector<HolonomicTrajectorySample>& samples);
    HolonomicTrajectory(std::vector<HolonomicTrajectorySample>&& samples);

    void CheckKinematics() const;

    /**
     * @brief Append a string representation of a holonomic trajectory to
     * an output stream. A string representation of a holonomic trajectory
     * is a json array of its trajectory segments' json representations.
     * 
     * @param stream the stream to append the string representation to
     * @param trajectory the holonomic trajectory
     * @return a reference to the given stream
     */
    friend std::ostream& operator<<(std::ostream& stream, const SwerveTrajectory& trajectory);
};
}