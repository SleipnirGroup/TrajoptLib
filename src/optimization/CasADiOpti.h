// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <casadi/casadi.hpp>

#include "optimization/OptiSys.h"

namespace trajopt {

class CasADiIterCallback : public Callback {
  // Data members
  double d;
public:
  // Constructor
  CasADiIterCallback(const std::string& name, double d,
             const Dict& opts=Dict()) : d(d) {
    construct(name, opts);
  }

  // Destructor
  ~CasADiIterCallback() override {}

  // Number of inputs and outputs
  casadi_int get_n_in() override { return 1;}
  casadi_int get_n_out() override { return 1;}

  // Initialize the object
  void init() override() {
    std::cout << "initializing object" << std::endl;
  }

  // Evaluate numerically
  std::vector<DM> eval(const std::vector<DM>& arg) const override {
    DM x = arg.at(0);
    DM f = sin(d*x);
    return {f};
  }
};

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
