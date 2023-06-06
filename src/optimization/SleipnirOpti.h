// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include <sleipnir/autodiff/Variable.hpp>
#include <sleipnir/optimization/Constraints.hpp>
#include <sleipnir/optimization/OptimizationProblem.hpp>

#include "optimization/OptiSys.h"

namespace trajopt {

struct SleipnirExpr {
  sleipnir::Variable expr;

  SleipnirExpr() = default;
  SleipnirExpr(const SleipnirExpr& expr) = default;
  explicit SleipnirExpr(sleipnir::Variable&& vari);
  SleipnirExpr(double value);  // NOLINT

  SleipnirExpr& operator=(const SleipnirExpr& b) = default;
  SleipnirExpr& operator=(SleipnirExpr&& b) = default;

  friend SleipnirExpr operator-(const SleipnirExpr& a);

  friend SleipnirExpr operator+(const SleipnirExpr& a, const SleipnirExpr& b);
  friend SleipnirExpr operator-(const SleipnirExpr& a, const SleipnirExpr& b);
  friend SleipnirExpr operator*(const SleipnirExpr& a, const SleipnirExpr& b);
  friend SleipnirExpr operator/(const SleipnirExpr& a, const SleipnirExpr& b);

  void operator+=(const SleipnirExpr& b);

  friend SleipnirExpr sin(const SleipnirExpr& a);
  friend SleipnirExpr cos(const SleipnirExpr& a);

  friend SleipnirExpr fmin(const SleipnirExpr& a, const SleipnirExpr& b);
  friend SleipnirExpr fmax(const SleipnirExpr& a, const SleipnirExpr& b);

  friend sleipnir::EqualityConstraints operator==(const SleipnirExpr& a,
                                                  const SleipnirExpr& b);
  friend sleipnir::InequalityConstraints operator>=(const SleipnirExpr& a,
                                                    const SleipnirExpr& b);
  friend sleipnir::InequalityConstraints operator<=(const SleipnirExpr& a,
                                                    const SleipnirExpr& b);
};

class SleipnirOpti {
 private:
  sleipnir::OptimizationProblem opti;

 public:
  trajopt::SleipnirExpr DecisionVariable();
  void Minimize(trajopt::SleipnirExpr&& objective);
  void Maximize(trajopt::SleipnirExpr&& objective);
  void SubjectTo(sleipnir::EqualityConstraints&& constraint);
  void SubjectTo(sleipnir::InequalityConstraints&& constraint);
  void SetInitial(trajopt::SleipnirExpr& expr, double value);
  void Solve();
  double SolutionValue(const trajopt::SleipnirExpr& expr) const;
};
}  // namespace trajopt

static_assert(OptiSys<trajopt::SleipnirExpr, trajopt::SleipnirOpti>);
