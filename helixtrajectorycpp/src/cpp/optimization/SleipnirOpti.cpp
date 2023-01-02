#ifdef OPTIMIZER_TYPE_SLEIPNIR

#include "optimization/SleipnirOpti.h"

#include <memory>
#include <vector>

#include <sleipnir/autodiff/Variable.hpp>
#include <sleipnir/optimization/Constraints.hpp>
#include <sleipnir/optimization/OptimizationProblem.hpp>

#include "optimization/HolonomicTrajectoryOptimizationProblem.h"
#include "optimization/SwerveTrajectoryOptimizationProblem.h"
#include "optimization/TrajectoryOptimizationProblem.h"

#include "DebugOptions.h"

namespace helixtrajectory {

SleipnirOpti::SleipnirOpti() : opti() {
}

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

void SleipnirOpti::SetInitial(sleipnir::VariableMatrix& expression, double value) {
    expression = value;
}

void SleipnirOpti::Solve() {
    static sleipnir::SolverConfig config;
    config.diagnostics = true;
    opti.Solve(config);
}

double SleipnirOpti::SolutionValue(const sleipnir::VariableMatrix& expression) const {
    return expression.Value(0);
}

// template class HolonomicTrajectoryOptimizationProblem<SleipnirOpti>;
// template class SwerveTrajectoryOptimizationProblem<SleipnirOpti>;
// template class TrajectoryOptimizationProblem<SleipnirOpti>;
}

#endif