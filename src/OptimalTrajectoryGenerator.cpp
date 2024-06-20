// Copyright (c) TrajoptLib contributors

#include "trajopt/OptimalTrajectoryGenerator.h"

#if defined(OPTIMIZER_BACKEND_CASADI)
#include "optimization/CasADiOpti.h"
#elif defined(OPTIMIZER_BACKEND_SLEIPNIR)
#include "optimization/SleipnirOpti.h"
#endif

#include "optimization/algorithms/SwerveDiscreteOptimal.h"

namespace trajopt {

expected<SwerveSolution, std::string> OptimalTrajectoryGenerator::Generate(
    const SwervePathBuilder& path, bool diagnostics, int64_t handle) {
#if defined(OPTIMIZER_BACKEND_CASADI)
  SwerveDiscreteOptimal<casadi::MX, CasADiOpti> problem(
#elif defined(OPTIMIZER_BACKEND_SLEIPNIR)
  SwerveDiscreteOptimal<SleipnirExpr, SleipnirOpti> problem(
#endif
      path.GetPath(), path.GetControlIntervalCounts(),
      path.CalculateSplineInitialGuessWithKinematicsAndConstraints(), handle);
  return problem.Generate(diagnostics);
}

}  // namespace trajopt
