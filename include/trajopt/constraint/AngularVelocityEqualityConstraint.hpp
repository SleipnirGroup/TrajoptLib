// Copyright (c) TrajoptLib contributors

#pragma once

#include <sleipnir/optimization/OptimizationProblem.hpp>

#include "trajopt/geometry/Pose2.hpp"
#include "trajopt/geometry/Translation2.hpp"
#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * Angular velocity equality constraint.
 */
class TRAJOPT_DLLEXPORT AngularVelocityEqualityConstraint {
 public:
  /**
   * Constructs an AngularVelocityEqualityConstraint.
   *
   * @param angularVelocity The angular velocity.
   */
  explicit AngularVelocityEqualityConstraint(double angularVelocity)
      : m_angularVelocity{angularVelocity} {}

  /**
   * Applies this constraint to the given problem.
   *
   * @param problem The optimization problem.
   * @param pose The robot's pose.
   * @param linearVelocity The robot's linear velocity.
   * @param angularVelocity The robot's angular velocity.
   */
  void Apply(sleipnir::OptimizationProblem& problem,
             [[maybe_unused]] const Pose2v& pose,
             [[maybe_unused]] const Translation2v& linearVelocity,
             const sleipnir::Variable& angularVelocity) {
    problem.SubjectTo(angularVelocity == m_angularVelocity);
  }

 private:
  double m_angularVelocity;
};

}  // namespace trajopt
