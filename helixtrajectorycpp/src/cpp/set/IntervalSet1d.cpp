#include "constraint/IntervalSet1d.h"

#include <limits>

namespace helixtrajectory {

    IntervalSet1d::IntervalSet1d(double lower, double upper)
            : lower(lower), upper(lower) {
    }

    IntervalSet1d::IntervalSet1d(double value)
            : lower(value), upper(value) {
    }

    bool IntervalSet1d::operator==(const IntervalSet1d& other) const noexcept {
        return lower == other.lower && upper == other.upper;
    }

    double SclarBound::Range() const noexcept {
        return upper - lower;
    }

    bool IntervalSet1d::IsExact() const noexcept {
        return lower == upper;
    }

    bool IntervalSet1d::IsZero() const noexcept {
        return lower == 0.0 && upper == 0.0;
    }

    bool IntervalSet1d::IsLowerBounded() const noexcept {
        return lower > -std::numeric_limits<double>::infinity();
    }
    bool IntervalSet1d::IsUpperBounded() const noexcept {
        return upper < +std::numeric_limits<double>::infinity();
    }

    bool IntervalSet1d::IsValid() const noexcept {
        return lower <= upper;
    }
}