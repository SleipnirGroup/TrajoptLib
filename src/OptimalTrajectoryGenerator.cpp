// Copyright (c) TrajoptLib contributors

#include "OptimalTrajectoryGenerator.h"

#if defined(OPTIMIZER_BACKEND_CASADI)
#include "optimization/CasADiOpti.h"
#define _OPTI_BACKEND CasADiOpti
#elif defined(OPTIMIZER_BACKEND_SLEIPNIR)
#include "optimization/SleipnirOpti.h"
#define _OPTI_BACKEND SleipnirOpti
#endif
#include "DebugOptions.h"
#include "InvalidPathException.h"
#include "drivetrain/SwerveDrivetrain.h"
// #include "optimization/SwerveTrajectoryOptimizationProblem.h"
#include "solution/SwerveSolution.h"

namespace trajopt {

SwerveSolution OptimalTrajectoryGenerator::Generate(
    const SwervePath& swervePath) {
  // if (!holonomicPath.IsValid()) {
  //   throw InvalidPathException("Cannot optimize an invalid path.");
  // }
  // SwerveTrajectoryOptimizationProblem<_OPTI_BACKEND> problem(swerveDrivetrain,
  //                                                            holonomicPath);
  // return problem.Generate();
  return SwerveSolution{};
}
}  // namespace trajopt
