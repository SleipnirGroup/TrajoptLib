#include "ScalarBound.h"

namespace helixtrajectory {

    ScalarBound::ScalarBound(double lowerBound, double upperBound)
            : lowerBound(lowerBound), upperBound(upperBound) {
    }

    bool ScalarBound::IsValid() const noexcept {
        return lowerBound <= upperBound;
    }
}