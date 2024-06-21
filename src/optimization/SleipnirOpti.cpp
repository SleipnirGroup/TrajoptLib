// Copyright (c) TrajoptLib contributors

#include "optimization/SleipnirOpti.hpp"

#include <vector>

#include <sleipnir/autodiff/Variable.hpp>
#include <sleipnir/optimization/Constraints.hpp>
#include <sleipnir/optimization/OptimizationProblem.hpp>

#include "optimization/Cancellation.hpp"

namespace trajopt {

SleipnirExpr::SleipnirExpr(sleipnir::Variable&& vari) : expr(std::move(vari)) {}

SleipnirExpr::SleipnirExpr(double value) : expr(value) {}

SleipnirExpr operator-(const SleipnirExpr& a) {
  return SleipnirExpr(-a.expr);
}

SleipnirExpr operator+(const SleipnirExpr& a, const SleipnirExpr& b) {
  return SleipnirExpr(a.expr + b.expr);
}

SleipnirExpr operator-(const SleipnirExpr& a, const SleipnirExpr& b) {
  return SleipnirExpr(a.expr - b.expr);
}

SleipnirExpr operator*(const SleipnirExpr& a, const SleipnirExpr& b) {
  return SleipnirExpr(a.expr * b.expr);
}

SleipnirExpr operator/(const SleipnirExpr& a, const SleipnirExpr& b) {
  return SleipnirExpr(a.expr / b.expr);
}

void SleipnirExpr::operator+=(const SleipnirExpr& b) {
  expr += b.expr;
}

SleipnirExpr sin(const SleipnirExpr& a) {
  return SleipnirExpr(sleipnir::sin(a.expr));
}

SleipnirExpr cos(const SleipnirExpr& a) {
  return SleipnirExpr(sleipnir::cos(a.expr));
}

SleipnirExpr fmax(const SleipnirExpr& a, const SleipnirExpr& b) {
  return SleipnirExpr(+0.5 * (1 + sleipnir::sign(b.expr - a.expr)) *
                          (b.expr - a.expr) +
                      a.expr);
}

SleipnirExpr fmin(const SleipnirExpr& a, const SleipnirExpr& b) {
  return SleipnirExpr(-0.5 * (1 + sleipnir::sign(b.expr - a.expr)) *
                          (b.expr - a.expr) +
                      b.expr);
}

SleipnirExpr abs(const SleipnirExpr& a) {
  return SleipnirExpr(sleipnir::abs(a.expr));
}

SleipnirExpr hypot(const SleipnirExpr& a, const SleipnirExpr& b) {
  return SleipnirExpr(sleipnir::hypot(a.expr, b.expr));
}

sleipnir::EqualityConstraints operator==(const SleipnirExpr& a,
                                         const SleipnirExpr& b) {
  return a.expr == b.expr;
}

sleipnir::InequalityConstraints operator>=(const SleipnirExpr& a,
                                           const SleipnirExpr& b) {
  return a.expr >= b.expr;
}

sleipnir::InequalityConstraints operator<=(const SleipnirExpr& a,
                                           const SleipnirExpr& b) {
  return a.expr <= b.expr;
}

trajopt::SleipnirExpr SleipnirOpti::DecisionVariable() {
  return SleipnirExpr(opti.DecisionVariable());
}

void SleipnirOpti::Minimize(trajopt::SleipnirExpr&& objective) {
  opti.Minimize(std::move(objective.expr));
}

void SleipnirOpti::Maximize(trajopt::SleipnirExpr&& objective) {
  opti.Maximize(std::move(objective.expr));
}

void SleipnirOpti::SubjectTo(sleipnir::InequalityConstraints&& relations) {
  opti.SubjectTo(std::move(relations));
}

void SleipnirOpti::SubjectTo(sleipnir::EqualityConstraints&& relations) {
  opti.SubjectTo(std::move(relations));
}

void SleipnirOpti::SetInitial(trajopt::SleipnirExpr& expr, double value) {
  expr.expr.SetValue(value);
}

void SleipnirOpti::AddIntermediateCallback(std::function<void()> callback) {
  callbacks.push_back(callback);
}

[[nodiscard]]
expected<void, std::string> SleipnirOpti::Solve(bool diagnostics) {
  GetCancellationFlag() = 0;
  opti.Callback([=, this](const sleipnir::SolverIterationInfo&) -> bool {
    for (auto& callback : callbacks) {
      callback();
    }
    return trajopt::GetCancellationFlag();
  });

  // tolerance of 1e-4 is 0.1 mm
  auto status = opti.Solve({.tolerance = 1e-4, .diagnostics = diagnostics});

  if (static_cast<int>(status.exitCondition) < 0 ||
      status.exitCondition ==
          sleipnir::SolverExitCondition::kCallbackRequestedStop) {
    return unexpected{std::string{sleipnir::ToMessage(status.exitCondition)}};
  }

  return {};
}

double SleipnirOpti::SolutionValue(const SleipnirExpr& expression) const {
  return expression.expr.Value();
}

}  // namespace trajopt
