#pragma once

#include "HolonomicPath.h"
#include "HolonomicTrajectory.h"
#include "SwerveDrivetrain.h"

namespace helixtrajectory {

    /**
     * @brief This trajectory generator class contains functions to generate time-optimal
     * trajectories for several drivetrain types.
     * 
     * @author Justin Babilino
     */
    class OptimalTrajectoryGenerator {
    public:
        /**
         * @brief Prevent instantiation
         */
        OptimalTrajectoryGenerator() = delete;

        /**
         * @brief Initializes and solves an optimization problem for a swerve drivetrain.
         * This function may throw an exception if the optimizer is unable to find the optizer
         * 
         * @param swerveDrivetrain the swerve drivetrain
         * @param holonomicPath the holonomic path
         * @return the optimized holonomic trajectory
         */
        static HolonomicTrajectory Generate(const SwerveDrivetrain& swerveDrivetrain, const HolonomicPath& holonomicPath);
    };
}