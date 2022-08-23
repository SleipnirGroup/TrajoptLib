#pragma once

#include "HolonomicDrivetrain.h"
#include "HolonomicPath.h"
#include "HolonomicTrajectory.h"

namespace helixtrajectory {

    class OptimalTrajectoryGenerator {
    public:
        static HolonomicTrajectory Generate(const HolonomicDrivetrain& holonomicDrivetrain, const HolonomicPath& holonomicPath);
    };
}