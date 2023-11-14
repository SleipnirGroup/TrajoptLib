#include "casadi/casadi.hpp"
#include "casadi/core/nlpsol.hpp"
#include "casadi/core/sparsity.hpp"
using namespace casadi;
class CasADiIterCallback : public Callback {
  // Data members
  casadi_int nx;
  casadi_int ng;
  casadi_int np;
public:
  // Constructor
  CasADiIterCallback(const std::string& name,
        casadi_int nx, casadi_int ng, casadi_int np,
             const Dict& opts=Dict()) : nx(nx), ng(ng), np(np) {
    construct(name, opts);
  }

  // Destructor
  ~CasADiIterCallback() override {}

  // Number of inputs and outputs
  casadi_int get_n_in() override { return 6;}
  casadi_int get_n_out() override { return 1;}
  Sparsity get_sparsity_in(casadi_int i) override {
    switch (static_cast<NlpsolOutput>(i)) {
    case NLPSOL_F:
      return Sparsity::scalar();
    case NLPSOL_X:
    case NLPSOL_LAM_X:
      return Sparsity::dense(nx);
    case NLPSOL_LAM_G:
    case NLPSOL_G:
        return Sparsity::dense(ng);
    case NLPSOL_LAM_P:
      return Sparsity::dense(np);
    case NLPSOL_NUM_OUT: break;
    }
    return Sparsity();
  }

//       /// Decision variables at the optimal solution (nx x 1)
//   NLPSOL_X,
//   /// Cost function value at the optimal solution (1 x 1)
//   NLPSOL_F,
// //   /// Constraints function at the optimal solution (ng x 1)
// //   NLPSOL_G,
// //   /// Lagrange multipliers for bounds on X at the solution (nx x 1)
// //   NLPSOL_LAM_X,
// //   /// Lagrange multipliers for bounds on G at the solution (ng x 1)
// //   NLPSOL_LAM_G,
// //   /// Lagrange multipliers for bounds on P at the solution (np x 1)
// //   NLPSOL_LAM_P,
// //   NLPSOL_NUM_OUT
// // };

//     function v = get_sparsity_in(self, i)

//       n = casadi.nlpsol_out(i);

//       if n=='f'

//         v =  casadi.Sparsity.scalar();

//       elseif strcmp(n,'x') || strcmp(n,'lam_x')

//         v = casadi.Sparsity.dense(self.nx);

//       elseif strcmp(n,'g') || strcmp(n,'lam_g')

//         v = casadi.Sparsity.dense(self.ng);

//       elseif strcmp(n,'lam_p')

//         v = casadi.Sparsity.dense(self.np);

//       else

//         v = casadi.Sparsity(0,0);

//       end
//  }

  // Initialize the object
  void init() override {
    std::cout << "initializing object" << std::endl;
  }

  // Evaluate numerically
  std::vector<DM> eval(const std::vector<DM>& arg) const override {
    std::cout << "eval" << std::endl;
    return {1};
  }
};