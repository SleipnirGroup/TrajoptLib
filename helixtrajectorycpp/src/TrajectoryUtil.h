#pragma once

#include <vector>

#include "casadi/casadi.hpp"

#include "Path.h"
#include "HolonomicPath.h"
#include "HolonomicTrajectory.h"

namespace helixtrajectory {

    /**
     * @brief slices all the rows or columns
     */
    const casadi::Slice ALL;
    /**
     * @brief slices the first 3 rows or columns
     */
    const casadi::Slice ZERO_ONE_TWO(0, 3);
    /**
     * @brief slices the second 3 rows or columns
     */
    const casadi::Slice THREE_FOUR_FIVE(3, 6);

    casadi::DM generateInitialTrajectory(const Path& path);
    // https://www.desmos.com/calculator/cqmc1tjtsv
    template<typename LineNumberType, typename PointNumberType>
    inline casadi::MX linePointDist(LineNumberType lineStartX, LineNumberType lineStartY, LineNumberType lineEndX, LineNumberType lineEndY,
            PointNumberType pointX, PointNumberType pointY) {
        casadi::MX lX = lineEndX - lineStartX;
        casadi::MX lY = lineEndY - lineStartY;
        casadi::MX vX = pointX - lineStartX;
        casadi::MX vY = pointY - lineStartY;
        casadi::MX dot = vX * lX + vY * lY;
        casadi::MX lNormSquared = lX * lX + lY * lY;
        casadi::MX t = dot / lNormSquared;
        casadi::MX tBounded = fmax(fmin(t, 1), 0);
        casadi::MX iX = (1 - tBounded) * lineStartX + tBounded * lineEndX;
        casadi::MX iY = (1 - tBounded) * lineStartY + tBounded * lineEndY;
        casadi::MX distSquared = (iX - pointX) * (iX - pointX) + (iY - pointY) * (iY - pointY);
        return distSquared;
    }

    void printHolonomicPath(const HolonomicPath& path);
    void printHolonomicTrajectory(const HolonomicTrajectory& trajectory);
}