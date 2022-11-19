#pragma once

#include <vector>

#include "trajectory/HolonomicState.h"
#include "trajectory/HolonomicTrajectorySample.h"

namespace helixtrajectory {

/**
 * @brief This class is the parent class for all types of trajectories. All trajectories
 * are a list of trajectory segments.
 * 
 * @author Justin Babilino
 */
class HolonomicTrajectory {
public:
    HolonomicState initialState;
    std::vector<HolonomicTrajectorySample> samples;

    HolonomicTrajectory(const HolonomicState& initialState, const std::vector<HolonomicTrajectorySample>& samples);
    // HolonomicTrajectory(std::vector<HolonomicTrajectorySample>&& samples);

    /**
     * @brief Append a string representation of a holonomic trajectory to
     * an output stream. A string representation of a holonomic trajectory
     * is a json array of its trajectory segments' json representations.
     * 
     * @param stream the stream to append the string representation to
     * @param trajectory the holonomic trajectory
     * @return a reference to the given stream
     */
    friend std::ostream& operator<<(std::ostream& stream, const HolonomicTrajectory& trajectory);
};
}