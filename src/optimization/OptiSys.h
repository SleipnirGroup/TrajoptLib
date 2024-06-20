// Copyright (c) TrajoptLib contributors

#pragma once

#include <functional>

template <typename Expr>
concept ExprSys = requires(Expr expr, const Expr constExpr, double num) {
  Expr();
  Expr(num);
  Expr(constExpr);
  expr = expr;

  -constExpr;
  constExpr + constExpr;
  constExpr - constExpr;
  constExpr* constExpr;
  constExpr / constExpr;

  expr += constExpr;

  expr = sin(constExpr);  // NOLINT
  expr = cos(constExpr);  // NOLINT

  expr = fmin(constExpr, constExpr);   // NOLINT
  expr = fmax(constExpr, constExpr);   // NOLINT
  expr = abs(constExpr);               // NOLINT
  expr = hypot(constExpr, constExpr);  // NOLINT
};

template <typename Expr, typename Opti>
concept OptiSys =
    ExprSys<Expr> &&
    requires(Expr expr, const Expr constExpr, Opti opti, const Opti constOpti,
             double num, std::function<void()> callback) {
      Opti();
      expr = opti.DecisionVariable();
      opti.Minimize(-expr);
      opti.Maximize(-expr);
      opti.SubjectTo(constExpr == constExpr);
      opti.SubjectTo(constExpr >= constExpr);
      opti.SubjectTo(constExpr <= constExpr);
      opti.SetInitial(expr, num);
      opti.Solve();
      // Add a `void()` callback to be called by the solver on every iteration.
      // This function can be called multiple times to add multiple callbacks.
      // This function needs to be called before opti.Solve() to take effect.
      opti.AddIntermediateCallback(callback);
      num = constOpti.SolutionValue(expr);
    };
