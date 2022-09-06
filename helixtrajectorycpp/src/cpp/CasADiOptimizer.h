#pragma once

#include <casadi/casadi.hpp>

namespace helixtrajectory {

    class CasADiOptimizer {
    public:
        using Expression = casadi::MX;

    private:
        casadi::Opti opti;
        casadi::OptiSol* solution;

    public:
        CasADiOptimizer() : opti(), solution(nullptr) {
        }

        Expression Variable();
        void Minimize(const Expression& objective);
        void SetInitial(const Expression& expression, double value);
        void Generate();
        double SolutionValue(const Expression& expression);
    };
}