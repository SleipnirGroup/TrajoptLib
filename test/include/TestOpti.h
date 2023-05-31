#pragma once

#include <vector>
#include <memory>
#include <cmath>

#include "optimization/OptiSys.h"

class TestOpti {
  double minimizeObjective = 0.0;
  double maximizeObjective = 0.0;
  bool isViolating = false;

 public:
  double DecisionVariable() {return 0.0;}

  void Minimize(double expr) {
    minimizeObjective = expr;
  }
  void Maximize(double expr) {
    maximizeObjective = expr;
  }

  void SubjectTo(bool constraint) {
    if (constraint == false) {
      isViolating = true;
    }
  }
  void SetInitial(double expr, double value) {}

  void Solve() {}

  double SolutionValue(double expr) const {
    return expr;
  }

  double GetMaximizeObjective() {
    return maximizeObjective;
  }
  double GetMinimizeObjective() {
    return minimizeObjective;
  }
  bool IsViolating() {
    return isViolating;
  }
};

static_assert(OptiSys<double, TestOpti>);