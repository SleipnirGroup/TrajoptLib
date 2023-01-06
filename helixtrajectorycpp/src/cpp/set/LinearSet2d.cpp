#include "set/LinearSet2d.h"

#include <cmath>
#include <optional>

#include "set/IntervalSet1d.h"
#include "set/RectangularSet2d.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

LinearSet2d::LinearSet2d(double theta)
        : theta(theta) {
}

std::optional<SolutionError> LinearSet2d::CheckVector(double xComp, double yComp, const SolutionTolerances& tolerances) const noexcept {
    if (std::abs(xComp * std::sin(theta) - yComp * std::cos(theta))
            > tolerances.errorMargin) {
        double rComp = std::hypot(xComp, yComp);
        double thetaComp = std::atan2(yComp, xComp);
        return SolutionError{fmt::format("(r, θ) = ({}, {})", rComp, thetaComp)};
    }
    return std::nullopt;
}

RectangularSet2d LinearSet2d::RBoundToRectangular(double theta, const IntervalSet1d& rBound) {
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