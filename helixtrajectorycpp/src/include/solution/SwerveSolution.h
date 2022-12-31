#pragma once

#include <vector>

#include "solution/HolonomicSolution.h"

namespace helixtrajectory {
struct SwerveSolution : HolonomicSolution {

    std::vector<std::vector<double>> moduleFX;
    std::vector<std::vector<double>> moduleFY;
};
}