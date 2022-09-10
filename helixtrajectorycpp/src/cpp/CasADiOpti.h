#pragma once

#include <casadi/casadi.hpp>

namespace helixtrajectory {

    class CasADiOpti {
    public:
        using Expression = casadi::MX;

    private:
        casadi::Opti opti;
        casadi::OptiSol* solution;

    public:
        CasADiOpti();

        Expression Variable();
        void Minimize(const Expression& objective);
        void SubjectTo(const Expression& relation);
        void SetInitial(const Expression& expression, double value);
        void Solve();
        double SolutionValue(const Expression& expression) const;
    };
}