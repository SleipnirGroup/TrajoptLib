// Copyright (c) TrajoptLib contributors

#pragma once

#include <cmath>
#include <functional>
#include <string>

#include <trajopt/expected>

#include "optimization/OptiSys.hpp"

class TestOpti {
  double minimizeObjective = 0.0;
  double maximizeObjective = 0.0;
  bool isViolating = false;

 public:
  double DecisionVariable() { return 0.0; }

  void Minimize(double expr) { minimizeObjective = expr; }
  void Maximize(double expr) { maximizeObjective = expr; }

  void SubjectTo(bool constraint) {
    if (constraint == false) {
      isViolating = true;
    }
  }
  void SetInitial(double expr, double value) {}

  trajopt::expected<void, std::string> Solve() { return {}; }

  double SolutionValue(double expr) const { return expr; }

  double GetMaximizeObjective() { return maximizeObjective; }
  double GetMinimizeObjective() { return minimizeObjective; }
  void AddIntermediateCallback(std::function<void()> callback);
};

static_assert(OptiSys<double, TestOpti>);
