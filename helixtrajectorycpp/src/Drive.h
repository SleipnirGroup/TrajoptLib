#pragma once

#include <casadi/casadi.hpp>

#include "Obstacle.h"

namespace helixtrajectory {

    class Drive {
    private:
        /**
         * @brief Gives expressions for the position of each bumper corner relative to the nonrotating robot
         * coordinate system with a certain heading. Returns a 2 x 4 matrix of position expressions.
         * Each column is a certain corner, with the bumper corner order specified in this class. The first row
         * contains the x-coordinates, and the second row contains the y-coordinates.
         * 
         * @param theta the instantaneous heading of the robot
         * @return a 2 x 4 matrix of positions where each column is a module and each row is a coordinate
         */
        const casadi::MX SolveBumperCornerPosition(const casadi::MX& x, const casadi::MX& y,
                const casadi::MX& theta, const ObstaclePoint& bumperCorner) const;
    protected:
        Drive(double mass, double moi, const Obstacle& bumpers);
    public:
        /**
         * @brief mass of the robot
         */
        double mass;
        /**
         * @brief moment of inertia of robot about axis of rotation (currently through
         * center of robot coordinate system)
         */
        double moi;
        /**
         * @brief the boundaries of the robot's bumpers, represented as an Obstacle. Bumpers really
         * are obstacles when you think about it. By including the safetyDistance property, bumpers
         * with extremely rounded corners or circular bumpers are possible.
         */
        Obstacle bumpers;

        /**
         * @brief Applies constraints that prevent the robot from getting too close to a given obstacle. 
         * 
         * @param opti the current optimizer
         * @param x (nTotal + 1) x 1 column vector of the x-coordinate of the robot for each sample point
         * @param y (nTotal + 1) x 1 column vector of the y-coordinate of the robot for each sample point
         * @param theta (nTotal + 1) x 1 column vector of the robot's heading for each sample point
         * @param nTotal the number of segments in this trajectory (number of sample points - 1)
         */
        void ApplyObstacleConstraints(casadi::Opti& opti, const casadi::MX& x, const casadi::MX& y,
                const casadi::MX& theta, const size_t nTotal, const std::vector<Obstacle>& obstacles) const;
    };
}