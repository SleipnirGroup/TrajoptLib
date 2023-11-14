#include "casadi/casadi.hpp"
#include "casadi/core/nlpsol.hpp"
using namespace casadi;
class CasADiIterCallback : public Callback {
  // Data members
  double d;
public:
  // Constructor
  CasADiIterCallback(const std::string& name, double d,
             const Dict& opts=Dict()) : d(d) {
    construct(name, opts);
  }

  // Destructor
  ~CasADiIterCallback() override {}

  // Number of inputs and outputs
  casadi_int get_n_in() override { return 6;}
  casadi_int get_n_out() override { return 1;}
  Sparsity get_sparsity_in(casadi_int i) {
    return Nlpsol::get_sparsity_out(i);
  }

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