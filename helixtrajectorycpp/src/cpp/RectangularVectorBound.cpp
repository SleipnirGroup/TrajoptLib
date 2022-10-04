#include "RectangularVectorBound.h"

namespace helixtrajectory {

    RectangularVectorBound::RectangularVectorBound(double leftBound, double rightBound, double lowerBound, double upperBound)
            : leftBound(leftBound), rightBound(rightBound), lowerBound(lowerBound), upperBound(upperBound) {
    }

    bool RectangularVectorBound::IsValid() const noexcept {
        return rightBound >= leftBound && upperBound >= lowerBound;
    }
}