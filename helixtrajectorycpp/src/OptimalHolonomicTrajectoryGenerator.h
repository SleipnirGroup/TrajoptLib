#pragma once

#include <memory>

#include "OptimalTrajectoryGenerator.h"
#include "Obstacle.h"
#include "HolonomicDrivetrain.h"
#include "HolonomicPath.h"
#include "HolonomicTrajectory.h"

namespace helixtrajectory {

    /**
     * @brief This is the trajectory generator for all holonomic drivetrains. It can be used
     * to optimize a trajectory for a swerve, mecanum, or omni drivetrain. This is possible since
     * the kinematics and dynamics of the robot as a system are equivalent.
     */
    class OptimalHolonomicTrajectoryGenerator : public OptimalTrajectoryGenerator {
    private:
        /**
         * @brief the holonomic drivetrain
         */
        const HolonomicDrivetrain& holonomicDrivetrain;
        /**
         * @brief the holonomic path
         */
        const HolonomicPath& holonomicPath;

        /**
         * @brief The 3 x (controlIntervalTotal + 1) matrix of robot velocity state per trajectory sample point.
         * Each column is a sample point. The first row is the x-component of velocity, the second row is
         * the y-component of velocity, and the third row is the angular velocity.
         */
        casadi::MX V;
        /**
         * @brief the 1 x (controlIntervalTotal + 1) vector of the robot's x-component of velocity per trajectory sample point
         */
        casadi::MX vx;
        /**
         * @brief the 1 x (controlIntervalTotal + 1) vector of the robot's y-component of velocity per trajectory sample point
         */
        casadi::MX vy;
        /**
         * @brief the 1 x (controlIntervalTotal + 1) vector of the robot's angular velocity per trajectory sample point
         */
        casadi::MX omega;

        /**
         * @brief The 3 x controlIntervalTotal matrix of robot acceleration state per sample segment
         * (between two sample points). Each column is a sample segment. The first row
         * is the x-component of acceleration, the second row is the y-component of acceleration,
         * and the third row is the angular acceleration.
         */
        casadi::MX U;
        /**
         * @brief the 1 x controlIntervalTotal vector of the robot's x-component of acceleration per trajectory sample segment
         */
        casadi::MX ax;
        /**
         * @brief the 1 x controlIntervalTotal vector of the robot's y-component of acceleration per trajectory sample segment
         */
        casadi::MX ay;
        /**
         * @brief the 1 x controlIntervalTotal vector of the robot's angular acceleration per trajectory sample segment
         */
        casadi::MX alpha;

        /**
         * @brief Applies the constraints that force the robot's motion to comply
         * with the list of holonomic waypoints provided. This may include constraints on
         * velocity and angular velocity.
         */
        void ApplyHolonomicPathConstraints();
    public:
        /**
         * @brief Constructs a new OptimalHolonomicTrajectoryGenerator with a holonomic drivetrain,
         * a holonomic path, and a list of obstacles.
         * 
         * @param drivetrain the holonomic drivetrain
         * @param path the holonomic path
         * @param obstacles the list of obstacles
         */
        OptimalHolonomicTrajectoryGenerator(const HolonomicDrivetrain& holonomicDrivetrain, const HolonomicPath& holonomicPath, const std::vector<Obstacle>& obstacles);

        virtual ~OptimalHolonomicTrajectoryGenerator();

        /**
         * @brief Optimizes the given path using IPOPT. Note this function call
         * may take a long time to complete. It may also fail, and throw a
         * CasadiException.
         * 
         * @return a holonomic trajectory
         */
        HolonomicTrajectory Generate();
    };
}