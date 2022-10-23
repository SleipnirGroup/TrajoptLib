#include "constraint/ScalarBound.h"

#include <limits>

namespace helixtrajectory {

    ScalarBound::ScalarBound(double lower, double upper)
            : lower(lower), upper(lower) {
    }

    ScalarBound::ScalarBound(double value)
            : lower(value), upper(value) {
    }

    ScalarBound::ScalarBound()
            : lower(-std::numeric_limits<double>::infinity()), upper(std::numeric_limits<double>::infinity()) {
    }

    bool ScalarBound::operator==(const ScalarBound& other) const noexcept {
        return lower == other.lower && upper == other.upper;
    }

    double SclarBound::Range() const noexcept {
        return upper - lower;
    }

    bool ScalarBound::IsExact() const noexcept {
        return lower == upper;
    }

    bool ScalarBound::IsZero() const noexcept {
        return lower == 0.0 && upper == 0.0;
    }

    bool ScalarBound::IsInfinite() const noexcept {
        return lower <= -std::numeric_limits<double>::infinity()
                && upper >= std::numeric_limits<double>::infinity();
    }

    bool ScalarBound::IsValid() const noexcept {
        return lower <= upper;
    }
}