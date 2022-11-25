#include "optimization/SleipnirOpti.h"

#include <vector>

#include <sleipnir/autodiff/Variable.hpp>
#include <sleipnir/optimization/Constraints.hpp>
#include <sleipnir/optimization/OptimizationProblem.hpp>

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

    void SleipnirOpti::SubjectTo(const sleipnir::VariableMatrix& relation) {
        opti.SubjectTo(relation);
    }

    void SleipnirOpti::SubjectTo(sleipnir::InequalityConstraints&& relations) {
        opti.SubjectTo(relations);
    }
    void SleipnirOpti::SubjectTo(sleipnir::EqualityConstraints&& relations) {
        opti.SubjectTo(relations);
    }

    void SleipnirOpti::SetInitial(const sleipnir::VariableMatrix& expression, double value) {
        expression = value;
    }

    void SleipnirOpti::Solve() {
        opti.Solve();
    }

    double Sleipnir::SolutionValue(const sleipnir::VariableMatrix& expression) const {
        return expression;
    }
}