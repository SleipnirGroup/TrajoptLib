#include "set/IntervalSet1d.h"

#include <limits>
#include <optional>

#include <fmt/format.h>

#include "solution/SolutionChecking.h"

namespace helixtrajectory {

IntervalSet1d::IntervalSet1d(double lower, double upper)
        : lower(lower), upper(upper) {
}

IntervalSet1d::IntervalSet1d(double value)
        : lower(value), upper(value) {
}

IntervalSet1d IntervalSet1d::R1() {
    return IntervalSet1d(
            -std::numeric_limits<double>::infinity(),
            +std::numeric_limits<double>::infinity());
}

bool IntervalSet1d::operator==(const IntervalSet1d& other) const noexcept {
    return lower == other.lower && upper == other.upper;
}

double IntervalSet1d::Range() const noexcept {
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

std::optional<SolutionError> IntervalSet1d::CheckScalar(double scalar, const SolutionTolerances& tolerances) const noexcept {
    if (scalar < lower || scalar > upper) {
        return SolutionError(fmt::format("{} is outside bound [{}, {}]", scalar, lower, upper));
    }
    return std::nullopt;
}

bool IntervalSet1d::IsValid() const noexcept {
    return lower <= upper;
}
}