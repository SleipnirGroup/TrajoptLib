#pragma once

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
         * @brief Destroy the Trajectory object
         */
        virtual ~Trajectory() = default;
    };
}