#pragma once

#include <limits>

#include "set/IntervalSet1d.h"
#include "set/RectangularSet2d.h"

namespace helixtrajectory {

class LinearSet2d {
public:
    double theta;

    LinearSet2d(double theta);

    void CheckVector(double x, double y) const;

    static RectangularSet2d TransformRBound(double theta, const IntervalSet1d& rBound);
};
}