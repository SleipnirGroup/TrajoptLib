// Copyright (c) TrajoptLib contributors

#include "trajopt/set/IntervalSet1d.h"

#include <limits>

namespace trajopt {

IntervalSet1d::IntervalSet1d(double lower, double upper)
    : lower(lower), upper(upper) {}

IntervalSet1d::IntervalSet1d(double value) : lower(value), upper(value) {}

IntervalSet1d IntervalSet1d::R1() {
  return IntervalSet1d(-std::numeric_limits<double>::infinity(),
                       +std::numeric_limits<double>::infinity());
}

IntervalSet1d IntervalSet1d::LessThan(double max) {
  return IntervalSet1d(-std::numeric_limits<double>::infinity(), max);
}

IntervalSet1d IntervalSet1d::GreaterThan(double min) {
  return IntervalSet1d(min, +std::numeric_limits<double>::infinity());
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

bool IntervalSet1d::IsValid() const noexcept {
  return lower <= upper;
}

}  // namespace trajopt
