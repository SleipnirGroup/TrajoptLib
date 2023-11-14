#include "casadi/casadi.hpp"
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
  casadi_int get_n_in() override { return 4;}
  casadi_int get_n_out() override { return 1;}

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