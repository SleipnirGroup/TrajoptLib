#pragma once

#include "HolonomicPath.h"
#include "HolonomicTrajectory.h"
#include "SwerveDrivetrain.h"

namespace helixtrajectory {

    class OptimalTrajectoryGenerator {
    public:
        static HolonomicTrajectory Generate(const SwerveDrivetrain& swerveDrivetrain, const HolonomicPath& holonomicPath);
    };
}