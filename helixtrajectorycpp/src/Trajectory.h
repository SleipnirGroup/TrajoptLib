#pragma once

#include <vector>

namespace helixtrajectory {

    /**
     * @brief A sample point cooresponding to the state of the robot at a certain timestamp
     * during the trajectory. Note that some information about the robot state is missing
     * since it can be inferred with simple calculations. For example, the rotation of each
     * swerve module is omitted. State is measured relative to the field coordinate system.
     */
    struct TrajectorySample {
        /**
         * @brief the amount of time since the beginning of the trajectory
         */
        double ts;
        /**
         * @brief the x-coordinate of the robot
         */
        double x;
        /**
         * @brief the y-coordinate of the robot
         */
        double y;
        /**
         * @brief the heading of the robot
         */
        double heading;
        /**
         * @brief the x-coordinate of the robot's velocity
         */
        double vx;
        /**
         * @brief the y-coordinate of the robot's velocity
         */
        double vy;
        /**
         * @brief the angular velocity of the robot
         */
        double omega;
    };

    /**
     * @brief A collection of sequential sample points that make up a trajectory.
     * This gives the trajectory follower the exact data it needs to make the robot
     * follow the generated path.
     */
    struct Trajectory {

        /**
         * @brief the list of sample points that make up this trajectory
         */
        std::vector<TrajectorySample> samples;

        /**
         * @brief Construct a new Trajectory object with a list of sample points.
         * 
         * @param samples the list of sample points that make up this trajectory
         */
        Trajectory(const std::vector<TrajectorySample>& samples);
    };
}