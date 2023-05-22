// Copyright (c) TrajoptLib contributors

#pragma once

#include <memory>
#include <vector>

#include "obstacle/Obstacle.h"
#include "optimization/TrajectoryOptimizationProblem.h"
#include "solution/HolonomicSolution.h"

namespace trajopt {

/**
 * @brief This is the trajectory generator for all holonomic drivetrains. It can
 * be used to optimize a trajectory for a swerve, mecanum, or omni drivetrain.
 * This is possible since the kinematics and dynamics of system as a whole are
 * equivalent.
 *
 * It is the job of extending classes to constrain the internal inverse
 * kinematics and dyanmics of the system. There are two main constraints thet
 * must be applied. First, an extending class must apply constraints that equate
 * the forces applied by all the motors to the net force and net torque of the
 * holonomic system, and the velocity of each motor must be related to the net
 * velocity of the system. Second, the class must apply constraints that prevent
 * motors from using more power than is available. A simple model may set a
 * maximum velocity and torque for each motor, but a more accurate model may use
 * the motor equation to prevent the voltages used by each motor from exceeding
 * the available voltage.
 */
template <typename Opti>
class HolonomicTrajectoryOptimizationProblem
    : public TrajectoryOptimizationProblem<Opti> {
 protected:
  using Expression = typename Opti::Expression;
  /**
   * @brief
   *
   */
  std::vector<Expression> vx;
  /**
   * @brief the 1 x (controlIntervalTotal + 1) vector of the robot's y-component
   * of velocity per trajectory sample point
   */
  std::vector<Expression> vy;
  /**
   * @brief the 1 x (controlIntervalTotal + 1) vector of the robot's angular
   * velocity per trajectory sample point
   */
  std::vector<Expression> omega;

  /**
   * @brief the 1 x controlIntervalTotal vector of the robot's x-component of
   * acceleration per trajectory sample segment
   */
  std::vector<Expression> ax;
  /**
   * @brief the 1 x controlIntervalTotal vector of the robot's y-component of
   * acceleration per trajectory sample segment
   */
  std::vector<Expression> ay;
  /**
   * @brief the 1 x controlIntervalTotal vector of the robot's angular
   * acceleration per trajectory sample segment
   */
  std::vector<Expression> alpha;

  /**
   * @brief Construct a new CasADi Holonomic Trajectory Optimization Problem
   * with a holonomic drivetrain and holonomic path.
   *
   * @param holonomicDrivetrain the holonomic drivetrain
   * @param HolonomicPath the holonomic path
   */
  explicit HolonomicTrajectoryOptimizationProblem(std::vector<size_t>&& ctrlIntCnts);

 private:
  /**
   * @brief Apply constraints that relate the first zeroth, first, and second
   * derivatives of position as a double integrator.
   *
   * @param opti the current optimizer
   * @param dt the time differential vector
   * @param X the position matrix (zeroth derivative of position)
   * @param V the velocity matrix (first derivative of position)
   * @param U the acceleration matrix (second derivative of position)
   */
  static void ApplyKinematicsConstraints(
      Opti& opti, const std::vector<Expression>& dt,
      const std::vector<Expression>& x, const std::vector<Expression>& y,
      const std::vector<Expression>& theta, const std::vector<Expression>& vx,
      const std::vector<Expression>& vy, const std::vector<Expression>& omega,
      const std::vector<Expression>& ax, const std::vector<Expression>& ay,
      const std::vector<Expression>& alpha);

  static void ApplyHolonomicConstraint(
      Opti& opti, const Expression& x, const Expression& y, const Expression& theta,
      const Expression& vx, const Expression& vy,
      const Expression& omega, const Expression& ax, const Expression& ay,
      const Expression& alpha, const HolonomicConstraint& constraint);
};
}  // namespace trajopt

#include "optimization/HolonomicTrajectoryOptimizationProblem.inc"
