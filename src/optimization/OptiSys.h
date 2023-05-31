#pragma once

template<typename Expr> 
concept ExprSys = requires (Expr expr, const Expr constExpr, double num) {
  Expr();
  Expr(num);
  Expr(constExpr);
  expr = expr;

  -constExpr;
  constExpr + constExpr;
  constExpr - constExpr;
  constExpr * constExpr;
  constExpr / constExpr;

  expr += constExpr;
  
  expr = sin(constExpr);
  expr = cos(constExpr);

  expr = fmin(constExpr, constExpr);
  expr = fmax(constExpr, constExpr);
};

template<typename Expr, typename Opti> 
concept OptiSys = ExprSys<Expr> and
    requires (Expr expr, const Expr constExpr, Opti opti, const Opti constOpti, double num) {
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