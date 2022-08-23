#include "OptimalTrajectoryGenerator.h"

#include <stdexcept>

#include "CasADiHolonomicTrajectoryOptimizationProblem.h"

namespace helixtrajectory {

    HolonomicTrajectory OptimalTrajectoryGenerator::Generate(const HolonomicDrivetrain& holonomicDrivetrain, const HolonomicPath& holonomicPath) {
        CasADiHolonomicTrajectoryOptimizationProblem problem(holonomicDrivetrain, holonomicPath);
        return problem.Generate();
    }
}