// Copyright (c) TrajoptLib contributors

#pragma once

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
  expr = atan2(constExpr, constExpr); // NOLINT

  expr = fmin(constExpr, constExpr);  // NOLINT
  expr = fmax(constExpr, constExpr);  // NOLINT
  expr = abs(constExpr); // NOLINT
  expr = sqrt(constExpr); // NOLINT
};

template <typename Expr, typename Opti>
concept OptiSys =
    ExprSys<Expr> && requires(Expr expr, const Expr constExpr, Opti opti,
                              const Opti constOpti, double num) {
      Opti();
      expr = opti.DecisionVariable();
      opti.Minimize(-expr);
      opti.Maximize(-expr);
      opti.SubjectTo(constExpr == constExpr);
      opti.SubjectTo(constExpr >= constExpr);
      opti.SubjectTo(constExpr <= constExpr);
      opti.SetInitial(expr, num);
      opti.Solve();
      num = constOpti.SolutionValue(expr);
    };
