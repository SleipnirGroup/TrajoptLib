// Copyright (c) TrajoptLib contributors

#include "trajopt/OptimalTrajectoryGenerator.hpp"

#include "optimization/algorithms/SwerveDiscreteOptimal.hpp"

namespace trajopt {

expected<SwerveSolution, std::string> OptimalTrajectoryGenerator::Generate(
    const SwervePathBuilder& path, bool diagnostics, int64_t handle) {
  SwerveDiscreteOptimal problem(path.GetPath(), path.GetControlIntervalCounts(),
                                path.CalculateInitialGuess(), handle);
  return problem.Generate(diagnostics);
}

}  // namespace trajopt
