#pragma once

#include <vector>

#include <sleipnir/autodiff/Variable.hpp>
#include <sleipnir/optimization/Constraints.hpp>
#include <sleipnir/optimization/OptimizationProblem.hpp>

namespace helixtrajectory {

    class SleipnirOpti {
    public:
        using Expression = sleipnir::VariableMatrix;

    private:
        sleipnir::OptimizationProblem opti;

    public:
        SleipnirOpti();

        Expression Variable();
        void Minimize(const Expression& objective);
        void SubjectTo(const Expression& relation);
        void SubjectTo(sleipnir::EqualityConstraints&& relations);
        void SubjectTo(sleipnir::InequalityConstraints&& relations);
        void SetInitial(Expression& expression, double value);
        void Solve();
        double SolutionValue(const Expression& expression) const;
    };
}