#include "PolarVectorBound.h"

#include <cmath>

namespace helixtrajectory {

    PolarVectorBound::PolarVectorBound(double magnitudeLowerBound, double magnitudeUpperBound,
            double angleLowerBound, double angleUpperBound)
            : magnitudeLowerBound(magnitudeLowerBound), magnitudeUpperBound(magnitudeUpperBound),
            angleLowerBound(angleLowerBound), angleUpperBound(angleUpperBound) {
    }

    bool PolarVectorBound::IsValid() const noexcept {
        return angleUpperBound >= angleLowerBound
                && angleLowerBound >= -M_PI
                && angleUpperBound <= +M_PI;
    }
}