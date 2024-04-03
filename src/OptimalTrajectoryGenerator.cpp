// Copyright (c) TrajoptLib contributors

#include "trajopt/OptimalTrajectoryGenerator.h"
#include <stdint.h>
#include "trajopt/expected"
#include "trajopt/path/SwervePathBuilder.h"

#if defined(OPTIMIZER_BACKEND_CASADI)
#include "optimization/CasADiOpti.h"
#define _OPTI_BACKEND casadi::MX, CasADiOpti
#elif defined(OPTIMIZER_BACKEND_SLEIPNIR)
#include "optimization/SleipnirOpti.h"
#define _OPTI_BACKEND SleipnirExpr, SleipnirOpti
#endif
#include "trajopt/drivetrain/SwerveDrivetrain.h"
#include "optimization/algorithms/SwerveDiscreteOptimal.h"
#include "trajopt/solution/SwerveSolution.h"

namespace trajopt {

expected<SwerveSolution, std::string> OptimalTrajectoryGenerator::Generate(
    const SwervePathBuilder& path, bool diagnostics, int64_t handle) {
  SwerveDiscreteOptimal<_OPTI_BACKEND> problem(path.GetPath(),
                                               path.GetControlIntervalCounts(),
                                               path.CalculateInitialGuess(),
                                               handle);
  return problem.Generate(diagnostics);

}
}  // namespace trajopt
