#pragma once

namespace helixtrajectory {

    /**
     * @brief This class is the parent class for all trajectory samples. It includes
     * the fields for position state and the time differential field.
     * 
     * @author Justin Babilino
     */
    class TrajectorySample {
    public:
        /**
         * @brief the duration of the control interval leading up to this sample
         */
        double intervalDuration;
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
         * @brief Destroy the Trajectory Sample object
         */
        virtual ~TrajectorySample() = default;

    protected:
        /**
         * @brief Construct a new Trajectory Sample object with the duration of the
         * control interval, position, and heading.
         * 
         * @param intervalDuration the duration of the control interval leading up to this sample
         * @param x the x-coordinate of the robot
         * @param y the y-coordinate of the robot
         * @param heading the heading of the robot
         */
        TrajectorySample(double intervalDuration, double x, double y, double heading);
    };
}