#pragma once

#include <limits>

#include "set/IntervalSet1d.h"

namespace helixtrajectory {

class LinearSet2d {
public:
    double theta;
    IntervalSet1d rBound;

    LinearSet2d(double theta, const IntervalSet1d& rBound =
            {-std::numeric_limits<double>::infinity(), +std::numeric_limits<double>::infinity()});
};
}