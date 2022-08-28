#pragma once

#include <iostream>
#include <vector>

#include "TrajectorySegment.h"
#include "HolonomicTrajectorySample.h"

namespace helixtrajectory {

    /**
     * @brief This class represents a segment in a holonomic trajectory. It contains a list
     * of holonomic trajectory sample points.
     * 
     * @author Justin Babilino
     */
    class HolonomicTrajectorySegment : public TrajectorySegment {
    public:
        /**
         * @brief the list of holonomic samples that make up this segment
         */
        std::vector<HolonomicTrajectorySample> holonomicSamples;

        /**
         * @brief Construct a new Holonomic Trajectory Segment object with
         * a list of holonomic samples.
         * 
         * @param holonomicSamples the list of holonomic samples
         */
        explicit HolonomicTrajectorySegment(const std::vector<HolonomicTrajectorySample>& holonomicSamples);

        /**
         * @brief Append a string representation of a holonomic trajectory segment
         * to an output stream. A string representation of a holonomic trajectory
         * is a json array of its trajectory samples' json representations.
         * 
         * @param stream the stream to append the string representation to
         * @param segment the holonomic trajectory sample
         * @return a reference to the given stream
         */
        friend std::ostream& operator<<(std::ostream& stream, const HolonomicTrajectorySegment& segment);
    };
}