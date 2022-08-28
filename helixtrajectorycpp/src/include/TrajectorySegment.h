#pragma once

namespace helixtrajectory {

    /**
     * @brief This class is the parent class for all trajectory segments, which
     * all contain a list of trajectory samples.
     * 
     * @author Justin Babilino
     */

    class TrajectorySegment {
    public:
        /**
         * @brief Destroy the Trajectory Segment object
         */
        virtual ~TrajectorySegment() = default;
    };
}