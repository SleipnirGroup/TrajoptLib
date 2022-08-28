#pragma once

#include <memory>

#include "CasADiTrajectoryOptimizationProblem.h"
#include "Obstacle.h"
#include "HolonomicDrivetrain.h"
#include "HolonomicPath.h"
#include "HolonomicTrajectory.h"

namespace helixtrajectory {

    /**
     * @brief This is the trajectory generator for all holonomic drivetrains. It can be used
     * to optimize a trajectory for a swerve, mecanum, or omni drivetrain. This is possible since
     * the kinematics and dynamics of system as a whole are equivalent.
     * 
     * It is the job of extending classes to constrain the internal inverse kinematics and
     * dyanmics of the system. There are two main constraints thet must be applied.
     * First, an extending class must apply constraints that equate the forces applied
     * by all the motors to the net force and net torque of the holonomic system, and the velocity
     * of each motor must be related to the net velocity of the system. Second, the class
     * must apply constraints that prevent motors from using more power than is available.
     * A simple model may set a maximum velocity and torque for each motor, but a more accurate
     * model may use the motor equation to prevent the voltages used by each motor from exceeding
     * the available voltage.
     */
    class CasADiHolonomicTrajectoryOptimizationProblem : public CasADiTrajectoryOptimizationProblem {
    protected:
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
         * @brief the entries of the V matrix, separated into individual trajectory segments
         */
        std::vector<casadi::MX> VSegments;
        /**
         * @brief the entries of the vx vector, separated into individual trajectory segments
         */
        std::vector<casadi::MX> vxSegments;
        /**
         * @brief the entries of the vx vector, separated into individual trajectory segments
         */
        std::vector<casadi::MX> vySegments;
        /**
         * @brief the entries of the omega vector, separated into individual trajectory segments
         */
        std::vector<casadi::MX> omegaSegments;

        /**
         * @brief the entries of the U matrix, separated into individual trajectory segments
         */
        std::vector<casadi::MX> USegments;

        /**
         * @brief Construct a new CasADi Holonomic Trajectory Optimization Problem
         * with a holonomic drivetrain and holonomic path.
         * 
         * @param holonomicDrivetrain the holonomic drivetrain
         * @param HolonomicPath the holonomic path
         */
        CasADiHolonomicTrajectoryOptimizationProblem(const HolonomicDrivetrain& holonomicDrivetrain, const HolonomicPath& holonomicPath);

    private:
        /**
         * @brief Apply the constraints that force the robot's motion to comply
         * with the list of holonomic waypoints provided. This function only
         * applies constraints on velocity and angular velocity.
         * 
         * @param opti the current optimizer
         * @param vxSegments the x-component of the robot's velocity for each sample point, divided into segments
         * @param vySegments the y-component of the robot's velocity for each sample point, divided into segments
         * @param omegaSegments the angular velocity of the robot for each sample point, divided into segments
         * @param holonomicPath the holonomic path to apply constraints for
         */
        static void ApplyHolonomicWaypointConstraints(casadi::Opti& opti,
                const std::vector<casadi::MX>& vxSegments, const std::vector<casadi::MX>& vySegments,
                const std::vector<casadi::MX>& omegaSegments, const HolonomicPath& holonomicPath);

        /**
         * @brief Apply constraints that relate the first zeroth, first, and second derivatives of
         * position as a double integrator.
         * 
         * @param opti the current optimizer
         * @param dt the time differential vector
         * @param X the position matrix (zeroth derivative of position)
         * @param V the velocity matrix (first derivative of position)
         * @param U the acceleration matrix (second derivative of position)
         */
        static void ApplyKinematicsConstraints(casadi::Opti& opti,
                const casadi::MX& dt, const casadi::MX& X, const casadi::MX& V, const casadi::MX& U);

        /**
         * @brief Applies the drivetrain-specific constraints to the optimizer. There are two
         * main constraints applied: first, constraints that correlate the forces applied by
         * the motors to the dyanmics of the holonomic system and second, constraints that
         * prevent motors from using more power than is available. A simple model may set a maximum
         * velocity and torque for each motor, but a more accurate model may use the motor equation
         * to prevent the voltages used by each motor from exceeding the available voltage.
         */
        // virtual void ApplyDynamicsConstraints() = 0;

        static HolonomicTrajectory ConstructTrajectory(const casadi::OptiSol& solution,
                const std::vector<casadi::MX>& dtSegments,
                const std::vector<casadi::MX>& xSegments, const std::vector<casadi::MX>& ySegments,
                const std::vector<casadi::MX>& thetaSegments, const std::vector<casadi::MX>& vxSegments,
                const std::vector<casadi::MX>& vySegments, const std::vector<casadi::MX>& omegaSegments);

    public:
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