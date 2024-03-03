// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>
#include <string>

#include <casadi/casadi.hpp>

#include "CasADiIterCallback.h"
#include "optimization/OptiSys.h"
#include "trajopt/expected"

namespace trajopt {

class CasADiOpti {
 private:
  casadi::Opti opti;
  std::optional<casadi::OptiSol> solution;

 public:
  casadi::MX DecisionVariable();
  void Minimize(const casadi::MX& objective);
  void Maximize(const casadi::MX& objective);
  void SubjectTo(const casadi::MX& constraint);
  void SetInitial(const casadi::MX& expr, double value);
  [[nodiscard]]
  expected<void, std::string> Solve(bool diagnostics = false);
  double SolutionValue(const casadi::MX& expr) const;
};
}  // namespace trajopt

static_assert(OptiSys<casadi::MX, trajopt::CasADiOpti>);
