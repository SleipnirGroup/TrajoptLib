#pragma once

#include <vector>

#include "casadi/casadi.hpp"

#include "Path.h"
#include "Trajectory.h"

namespace helixtrajectory {

    const casadi::Slice all;
    const casadi::Slice zeroOneTwo(0, 3);
    const casadi::Slice threeFourFive(3, 6);

    void linspace(casadi::DM& x, size_t row, double start, double end, double n);

    casadi::DM generateInitialTrajectory(const Path& path, size_t nPerTrajectorySegment);
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

    void printPath(const Path& path);
    void printTrajectory(const Trajectory& trajectory);
}