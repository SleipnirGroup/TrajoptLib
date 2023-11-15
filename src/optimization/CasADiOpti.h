// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <casadi/casadi.hpp>

#include "CasADiIterCallback.h"
#include "optimization/OptiSys.h"

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
  void Solve();
  double SolutionValue(const casadi::MX& expr) const;
};
}  // namespace trajopt

static_assert(OptiSys<casadi::MX, trajopt::CasADiOpti>);
