// Copyright (c) TrajoptLib contributors

#ifdef OPTIMIZER_BACKEND_CASADI
#include "optimization/CasADiOpti.h"

#include <casadi/casadi.hpp>
#include <casadi/core/exception.hpp>
#include <casadi/core/generic_matrix.hpp>
#include <casadi/core/mx.hpp>

#include "DebugOptions.h"
#include "optimization/Cancellation.h"
#include "optimization/CasADiIterCallback.h"
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
  GetCancellationFlag() = 0;
  const auto callback =
      new const CasADiIterCallback("f", opti.nx(), opti.ng(), opti.np());
  auto pluginOptions = casadi::Dict();
  pluginOptions["iteration_callback"] = *callback;
#ifndef DEBUG_OUTPUT
  auto pluginOptions = casadi::Dict();
  pluginOptions["ipopt.print_level"] = 0;
  pluginOptions["print_time"] = 0;
  pluginOptions["ipopt.sb"] = "yes";
#endif

  // I don't try-catch this next line since it should always work.
  // I'm assuming the dynamic lib is on the path and casadi can find it.
  opti.solver("ipopt", pluginOptions);
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
