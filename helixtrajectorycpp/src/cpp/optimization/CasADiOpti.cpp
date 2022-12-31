// #include "optimization/CasADiOpti.h"

// #include "DebugOptions.h"

// namespace helixtrajectory {

//     CasADiOpti::CasADiOpti() : opti(), solution(nullptr) {
//     }

//     casadi::MX CasADiOpti::Variable() {
//         return opti.variable();
//     }

//     void CasADiOpti::Minimize(const casadi::MX& objective) {
//         opti.minimize(objective);
//     }

//     void CasADiOpti::SubjectTo(const casadi::MX& constraint) {
//         opti.subject_to(constraint);
//     }

//     void CasADiOpti::SetInitial(const casadi::MX& expression, double value) {
//         opti.set_initial(expression, value);
//     }

//     void CasADiOpti::Solve() {
// #ifdef DEBUG_OUTPUT
//         // I don't try-catch this next line since it should always work.
//         // I'm assuming the dynamic lib is on the path and casadi can find it.
//         opti.solver("ipopt");
//         std::cout << "Located IPOPT Plugin" << std::endl;
// #else
//         auto pluginOptions = casadi::Dict();
//         pluginOptions["ipopt.print_level"] = 0;
//         pluginOptions["print_time"] = 0;
//         pluginOptions["ipopt.sb"] = "yes";
//         opti.solver("ipopt", pluginOptions);
// #endif
//         solution = new casadi::OptiSol(opti.solve());
//     }

//     double CasADiOpti::SolutionValue(const casadi::MX& expression) const {
//         if (solution != nullptr) {
//             return static_cast<double>(solution->value(expression));
//         } else {
//             throw "Solution not generated properly";
//         }
//     }
// }