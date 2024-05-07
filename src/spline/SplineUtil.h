// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include "CubicHermitePoseSplineHolonomic.h"
#include "trajopt/path/InitialGuessPoint.h"

namespace trajopt {

std::vector<trajopt::CubicHermitePoseSplineHolonomic>
CubicControlVectorsFromWaypoints(
    const std::vector<std::vector<trajopt::InitialGuessPoint>>
        initialGuessPoints);

}  // namespace trajopt
