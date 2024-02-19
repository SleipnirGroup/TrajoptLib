// Copyright (c) TrajoptLib contributors

#pragma once

#include <string>
#include <vector>

#include <casadi/casadi.hpp>
#include <casadi/core/nlpsol.hpp>
#include <casadi/core/sparsity.hpp>

#include "optimization/Cancellation.h"

namespace trajopt {
using namespace casadi;
class CasADiIterCallback : public Callback {
  // Data members
  casadi_int nx;
  casadi_int ng;
  casadi_int np;

 public:
  // Constructor
  CasADiIterCallback(const std::string& name, casadi_int nx, casadi_int ng,
                     casadi_int np, const Dict& opts = Dict())
      : nx(nx), ng(ng), np(np) {
    construct(name, opts);
  }

  // Destructor
  ~CasADiIterCallback() override = default;

  // Number of inputs and outputs
  // boilerplate for us, since we don't use the inputs.
  casadi_int get_n_in() override { return 6; }
  casadi_int get_n_out() override { return 1; }
  Sparsity get_sparsity_in(casadi_int i) override {
    switch (static_cast<NlpsolOutput>(i)) {
      case NLPSOL_F:
        return Sparsity::scalar();
      case NLPSOL_X:
      case NLPSOL_LAM_X:
        return Sparsity::dense(nx);
      case NLPSOL_LAM_G:
      case NLPSOL_G:
        return Sparsity::dense(ng);
      case NLPSOL_LAM_P:
        return Sparsity::dense(np);
      case NLPSOL_NUM_OUT:
        break;
    }
    return Sparsity();
  }

  // Evaluate numerically
  std::vector<DM> eval(const std::vector<DM>& arg) const override {
    int flag = trajopt::GetCancellationFlag();
    return {static_cast<double>(flag)};
  }
};
}  // namespace trajopt
