// Copyright (c) TrajoptLib contributors

#ifdef OPTIMIZER_BACKEND_CASADI
#include "optimization/CasADiOpti.h"
#include "optimization/CasADiIterCallback.h"

#include <casadi/casadi.hpp>
#include <casadi/core/exception.hpp>
#include <casadi/core/generic_matrix.hpp>
#include <casadi/core/mx.hpp>

#include "DebugOptions.h"
namespace trajopt {
casadi::MX CasADiOpti::DecisionVariable() {
  return opti.variable();
}
void CasADiOpti::Minimize(const casadi::MX& objective) {
  opti.minimize(objective);
}
void CasADiOpti::SubjectTo(const casadi::MX& constraint) {
  opti.subject_to(constraint);
}
void CasADiOpti::SetInitial(const casadi::MX& expression, double value) {
  opti.set_initial(expression, value);
}
void CasADiOpti::Solve() {
#ifdef DEBUG_OUTPUT
  // I don't try-catch this next line since it should always work.
  // I'm assuming the dynamic lib is on the path and casadi can find it.
  auto pluginOptions = casadi::Dict();
  pluginOptions["ipopt.iteration_callback"] = MyCallback("f", 0.5);
  opti.solver("ipopt");
#else
  auto pluginOptions = casadi::Dict();
  pluginOptions["ipopt.print_level"] = 0;
  pluginOptions["print_time"] = 0;
  pluginOptions["ipopt.sb"] = "yes";
  opti.solver("ipopt", pluginOptions);
#endif

  solution = opti.solve();
}
double CasADiOpti::SolutionValue(const casadi::MX& expression) const {
  if (solution) {
    try {
      return static_cast<double>(solution->value(expression));
    } catch (...) {
      return 0.0;
    }
  } else {
    throw std::runtime_error("Solution not generated properly");
  }
}
}  // namespace trajopt
#endif
