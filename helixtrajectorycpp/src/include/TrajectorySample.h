#pragma once

namespace helixtrajectory {
    class TrajectorySample {
    public:
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

        virtual ~TrajectorySample();
    protected:
        TrajectorySample(double x, double y, double heading);
    };
}