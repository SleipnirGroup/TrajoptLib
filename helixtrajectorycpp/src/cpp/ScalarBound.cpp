#include "ScalarBound.h"

namespace helixtrajectory {

    ScalarBound::ScalarBound(double lower, double upper)
            : lower(lower), upper(lower) {
    }

    bool ScalarBound::IsValid() const noexcept {
        return lower <= upper;
    }
}