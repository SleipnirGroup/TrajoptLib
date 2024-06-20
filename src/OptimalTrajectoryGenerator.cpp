// Copyright (c) TrajoptLib contributors

#include "trajopt/OptimalTrajectoryGenerator.h"

#if defined(OPTIMIZER_BACKEND_SLEIPNIR)
#include "optimization/SleipnirOpti.h"
#endif

#include "optimization/algorithms/SwerveDiscreteOptimal.h"

namespace trajopt {

expected<SwerveSolution, std::string> OptimalTrajectoryGenerator::Generate(
    const SwervePathBuilder& path, bool diagnostics, int64_t handle) {
#if defined(OPTIMIZER_BACKEND_SLEIPNIR)
  SwerveDiscreteOptimal<SleipnirExpr, SleipnirOpti> problem(
#endif
      path.GetPath(), path.GetControlIntervalCounts(),
      path.CalculateInitialGuess(), handle);
  return problem.Generate(diagnostics);
}

}  // namespace trajopt
