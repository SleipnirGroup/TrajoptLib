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
         * @brief Construct a new Trajectory object with a list of segments.
         * 
         * @param samples the list of segments that make up this trajectory
         */
        HolonomicTrajectory(const std::vector<HolonomicTrajectorySegment>& holonomicSegments);

        friend std::ostream& operator<<(std::ostream& stream, const HolonomicTrajectory& trajectory);
    };
}