#pragma once

#include <iostream>
#include <vector>

#include "HolonomicTrajectorySegment.h"
#include "Trajectory.h"

namespace helixtrajectory {

    /**
     * @brief A type of trajectory used for holonomic drivetrain robots. It includes
     * position and velocity state that can be used for trajectory following.
     */
    class HolonomicTrajectory : Trajectory {
    public:
        /**
         * @brief the list of segments that make up this trajectory
         */
        std::vector<HolonomicTrajectorySegment> holonomicSegments;

        /**
         * @brief Construct a new Trajectory object with a list of holonomic segments.
         * 
         * @param samples the list of holonomic segments that make up this trajectory
         */
        explicit HolonomicTrajectory(const std::vector<HolonomicTrajectorySegment>& holonomicSegments);

        /**
         * @brief Append a string representation of a holonomic trajectory to
         * an output stream. A string representation of a holonomic trajectory
         * is a json array of its trajectory segments' json representations.
         * 
         * @param stream the stream to append the string representation to
         * @param sample the holonomic trajectory sample
         * @return a reference to the given stream
         */
        friend std::ostream& operator<<(std::ostream& stream, const HolonomicTrajectory& trajectory);
    };
}