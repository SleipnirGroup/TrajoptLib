#pragma once

#include <vector>

#include "trajectory/TrajectorySample.h"

namespace helixtrajectory {

    /**
     * @brief This class is the parent class for all types of trajectories. All trajectories
     * are a list of trajectory segments.
     * 
     * @author Justin Babilino
     */
    class Trajectory {
    public:
        /**
         * @brief the list of segments that make up this trajectory
         */
        std::vector<TrajectorySample> samples;

        Trajectory(const std::vector<TrajectorySample>& samples);

        /**
         * @brief Append a string representation of a holonomic trajectory to
         * an output stream. A string representation of a holonomic trajectory
         * is a json array of its trajectory segments' json representations.
         * 
         * @param stream the stream to append the string representation to
         * @param trajectory the holonomic trajectory
         * @return a reference to the given stream
         */
        friend std::ostream& operator<<(std::ostream& stream, const Trajectory& trajectory);
    };
}