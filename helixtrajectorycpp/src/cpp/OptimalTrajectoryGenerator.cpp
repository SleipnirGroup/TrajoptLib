#include "OptimalTrajectoryGenerator.h"

#include <stdexcept>

#include "CasADiSwerveTrajectoryOptimizationProblem.h"
#include "SwerveDrivetrain.h"

namespace helixtrajectory {

    HolonomicTrajectory OptimalTrajectoryGenerator::Generate(const SwerveDrivetrain& swerveDrivetrain, const HolonomicPath& holonomicPath) {
        CasADiSwerveTrajectoryOptimizationProblem problem(swerveDrivetrain, holonomicPath);
        return problem.Generate();
    }
}