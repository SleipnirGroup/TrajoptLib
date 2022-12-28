#pragma once

#include <vector>

#include "solution/HolonomicSolution.h"

struct SwerveSolution {

    std::vector<std::vector<double>> moduleFX;
    std::vector<std::vector<double>> moduleFY;
};