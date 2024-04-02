// Copyright (c) TrajoptLib contributors

#include "trajopt/OptimalTrajectoryGenerator.h"
#include <stdint.h>

#include "trajopt/path/SwervePathBuilder.h"

#if defined(OPTIMIZER_BACKEND_CASADI)
#include "optimization/CasADiOpti.h"
#define _OPTI_BACKEND casadi::MX, CasADiOpti
#elif defined(OPTIMIZER_BACKEND_SLEIPNIR)
#include "optimization/SleipnirOpti.h"
#define _OPTI_BACKEND SleipnirExpr, SleipnirOpti
#endif
#include "DebugOptions.h"
#include "trajopt/InvalidPathException.h"
#include "trajopt/drivetrain/SwerveDrivetrain.h"
#include "optimization/algorithms/SwerveDiscreteOptimal.h"
#include "trajopt/solution/SwerveSolution.h"

namespace trajopt {

SwerveSolution OptimalTrajectoryGenerator::Generate(
    const SwervePathBuilder& path, uint32_t handle) {
  SwerveDiscreteOptimal<_OPTI_BACKEND> problem(path.GetPath(),
                                               path.GetControlIntervalCounts(),
                                               path.CalculateInitialGuess(),
                                               handle);
  return problem.Generate();
}
}  // namespace trajopt
