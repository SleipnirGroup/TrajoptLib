#pragma once

template<typename Expr, typename Opti> 
concept OptiSys = requires (Expr expr, Opti opti, double num) {
  Expr();
  Expr(num);
  expr = expr;

  expr = -expr;
  expr = expr + expr;
  expr = expr - expr;
  expr = expr * expr;
  expr = expr / expr;

  expr += expr;
  
  sin(expr);
  cos(expr);

  Opti();
  expr = opti.DecisionVariable();
  opti.Minimize(expr);
  opti.Maximize(expr);
  opti.SubjectTo(expr == expr);
  opti.SubjectTo(expr >= expr);
  opti.SubjectTo(expr <= expr);
  opti.SetInitial(expr, num);
  opti.Solve();
  num = opti.SolutionValue(expr);
};