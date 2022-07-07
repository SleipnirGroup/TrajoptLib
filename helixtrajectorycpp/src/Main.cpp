#include <casadi/casadi.hpp>
#include <iostream>

int main() {
    using namespace casadi;
    Opti opti;
    MX x = opti.variable(3);
    MX y = x / 2;
    MX z = fmin(x, 2.0);
    opti.minimize(x(0)*x(0) + 2*x(0) - 1 + x(1)*x(1) + 2*x(1) + x(2)*x(2) - 5*x(2));
    opti.solver("ipopt");
    OptiSol solution = opti.solve();
    std::cout << solution.value(x);
    std::cout << solution.value(y);
}