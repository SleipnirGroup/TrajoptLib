// Copyright (c) TrajoptLib contributors

#pragma once

#include <string>
#include <vector>

#include <casadi/casadi.hpp>
#include <casadi/core/nlpsol.hpp>
#include <casadi/core/sparsity.hpp>

#include "trajopt/cancellation/Cancellation.h"

namespace trajopt {
class CasADiIterCallback : public casadi::Callback {
  // Data members
  casadi::casadi_int nx;
  casadi::casadi_int ng;
  casadi::casadi_int np;

 public:
  // Constructor
  CasADiIterCallback(const std::string& name, casadi::casadi_int nx,
                     casadi::casadi_int ng, casadi::casadi_int np,
                     const casadi::Dict& opts = Dict())
      : nx(nx), ng(ng), np(np) {
    construct(name, opts);
  }

  // Destructor
  ~CasADiIterCallback() override {}

  // Number of inputs and outputs
  // boilerplate for us, since we don't use the inputs.
  casadi::casadi_int get_n_in() override { return 6; }
  casadi::casadi_int get_n_out() override { return 1; }
  casadi::Sparsity get_sparsity_in(casadi::casadi_int i) override {
    switch (static_cast<casadi::NlpsolOutput>(i)) {
      case NLPSOL_F:
        return casadi::Sparsity::scalar();
      case NLPSOL_X:
      case NLPSOL_LAM_X:
        return casadi::Sparsity::dense(nx);
      case NLPSOL_LAM_G:
      case NLPSOL_G:
        return casadi::Sparsity::dense(ng);
      case NLPSOL_LAM_P:
        return casadi::Sparsity::dense(np);
      case NLPSOL_NUM_OUT:
        break;
    }
    return casadi::Sparsity();
  }

  // Evaluate numerically
  std::vector<casadi::DM> eval(
      const std::vector<casadi::DM>& arg) const override {
    int flag = trajopt::GetCancellationFlag();
    return {static_cast<double>(flag)};
  }
};
}  // namespace trajopt
