#pragma once

#include <cmath>
#include <limits>

#include "set/IntervalSet1d.h"
#include "set/RectangularSet2d.h"

namespace helixtrajectory {

class LinearSet2d {
public:
    double theta;

    LinearSet2d(double theta);

    template<typename Expression>
    decltype(Expression() == Expression()) GetConstraint(const Expression& x, const Expression& y) const {
        double sinTheta = std::sin(theta);
        double cosTheta = std::cos(theta);
        return x * sinTheta == y * cosTheta;
    }

    void CheckVector(double x, double y) const;

    static RectangularSet2d TransformRBound(double theta, const IntervalSet1d& rBound);
};
}