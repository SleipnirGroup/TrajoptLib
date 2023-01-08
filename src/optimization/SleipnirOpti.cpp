// Copyright (c) TrajoptLib contributors

#ifdef OPTIMIZER_BACKEND_SLEIPNIR
#include "optimization/SleipnirOpti.h"

#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include <sleipnir/autodiff/Variable.hpp>
#include <sleipnir/optimization/Constraints.hpp>
#include <sleipnir/optimization/OptimizationProblem.hpp>

#include "DebugOptions.h"
#include "optimization/HolonomicTrajectoryOptimizationProblem.h"
#include "optimization/SwerveTrajectoryOptimizationProblem.h"
#include "optimization/TrajectoryOptimizationProblem.h"
namespace trajopt {
SleipnirOpti::SleipnirOpti() : opti() {}
sleipnir::VariableMatrix SleipnirOpti::Variable() {
  return opti.DecisionVariable();
}
void SleipnirOpti::Minimize(const sleipnir::VariableMatrix& objective) {
  opti.Minimize(objective);
}
// void SleipnirOpti::SubjectTo(const sleipnir::VariableMatrix& relation) {
//     opti.SubjectTo(relation);
// }
void SleipnirOpti::SubjectTo(sleipnir::InequalityConstraints&& relations) {
  opti.SubjectTo(std::move(relations));
}
void SleipnirOpti::SubjectTo(sleipnir::EqualityConstraints&& relations) {
  opti.SubjectTo(std::move(relations));
}
void SleipnirOpti::SetInitial(sleipnir::VariableMatrix& expression,
                              double value) {
  expression = value;
}
void SleipnirOpti::Solve() {
  static sleipnir::SolverConfig config;
  config.diagnostics = true;
  config.maxIterations = std::numeric_limits<int>::max();
  opti.Solve(config);
}
double SleipnirOpti::SolutionValue(
    const sleipnir::VariableMatrix& expression) const {
  return expression.Value(0);
}
// template class HolonomicTrajectoryOptimizationProblem<SleipnirOpti>;
// template class SwerveTrajectoryOptimizationProblem<SleipnirOpti>;
// template class TrajectoryOptimizationProblem<SleipnirOpti>;
}  // namespace trajopt
#endif
