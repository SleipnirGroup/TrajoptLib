#include "set/LinearSet2d.h"

#include <cmath>

#include <fmt/format.h>

#include "IncompatibleTrajectoryException.h"
#include "set/IntervalSet1d.h"
#include "set/RectangularSet2d.h"
#include "TestUtil.h"

namespace helixtrajectory {

LinearSet2d::LinearSet2d(double theta)
        : theta(theta) {
}

void LinearSet2d::CheckVector(double x, double y) const {
    bool check = GetConstraint<LooseDouble>(x, y);
    if (!check) {
        throw IncompatibleTrajectoryException(
                fmt::format("({}, {}) is not on line defined by θ = {}", x, y, theta));
    }
}

RectangularSet2d LinearSet2d::TransformRBound(double theta, const IntervalSet1d& rBound) {
    double sinTheta = sin(theta);
    double cosTheta = cos(theta);
    if (sinTheta > abs(cosTheta)) { // y > |x|, up cone
        double lowerVectorY = rBound.lower * sinTheta;
        double upperVectorY = rBound.upper * sinTheta;
        return RectangularSet2d(IntervalSet1d::R1(), {lowerVectorY, upperVectorY});
    } else if (sinTheta < -abs(cosTheta)) { // y < -|x|, down cone
        double lowerVectorY = rBound.upper * sinTheta;
        double upperVectorY = rBound.lower * sinTheta;
        return RectangularSet2d(IntervalSet1d::R1(), {lowerVectorY, upperVectorY});
    } else if (cosTheta >= abs(sinTheta)) { // x ≥ |y|, right cone
        double lowerVectorX = rBound.lower * cosTheta;
        double upperVectorX = rBound.upper * cosTheta;
        return RectangularSet2d({lowerVectorX, upperVectorX}, IntervalSet1d::R1());
    } else /*if (cosTheta <= -abs(sinTheta))*/ { // x ≤ -|y|, left cone
        double lowerVectorX = rBound.upper * cosTheta;
        double upperVectorX = rBound.lower * cosTheta;
        return RectangularSet2d({lowerVectorX, upperVectorX}, IntervalSet1d::R1());
    }
}
}