// Copyright (c) TrajoptLib contributors

#pragma once

#include <limits>

#include "trajopt/SymbolExports.h"

namespace trajopt {

/**
 * This class represents a bounded region of abstract 1D space. The bound
 * is specified by an inclusive inequality between two numbers.
 */
struct TRAJOPT_DLLEXPORT IntervalSet1d {
  /// The lower bound.
  double lower;

  /// The upper bound.
  double upper;

  /**
   * Construct a Scalar Bound between a lower and upper bound.
   *
   * @param lower The lower bound.
   * @param upper The upper bound.
   */
  constexpr IntervalSet1d(double lower, double upper)
      : lower(lower), upper(upper) {}

  /**
   * Construct a Scalar Bound that represents the interval [value, value].
   *
   * @param value the value to bound the number between.
   */
  constexpr IntervalSet1d(double value)
      : lower(value), upper(value) {}  // NOLINT

  constexpr IntervalSet1d() = default;

  /**
   * Returns an IntervalSet1d spanning R¹.
   */
  static constexpr IntervalSet1d R1() {
    return IntervalSet1d(-std::numeric_limits<double>::infinity(),
                         +std::numeric_limits<double>::infinity());
  }

  /**
   * Returns an IntervalSet1d that contains all the real numbers less than or
   * equal to a maximum value
   *
   * @param max the maximum value
   * @return [-∞, max]
   */
  static constexpr IntervalSet1d LessThan(double max) {
    return IntervalSet1d(-std::numeric_limits<double>::infinity(), max);
  }

  /**
   * Returns an IntervalSet1d that contains all the real numbers greater than or
   * equal to a minimum value
   *
   * @param min the minimum value
   * @return [min, ∞]
   */
  static constexpr IntervalSet1d GreaterThan(double min) {
    return IntervalSet1d(min, +std::numeric_limits<double>::infinity());
  }

  /**
   * Check if this scalar bound is equivalent to another scalar bound.
   * Two scalar bounds are equivalent if their upper and lower bounds are equal.
   * This operator can also be used to check if this bound is equal to a number,
   * meaning the upper and lower bounds both equal the given number.
   *
   * @param other the other scalar bound
   * @return lower == other.lower && upper == other.upper
   */
  constexpr bool operator==(const IntervalSet1d& other) const = default;

  /**
   * Calculate the range of this scalar bound, which is the difference
   * between the upper and lower bounds. An infinite scalar bound has a range
   * of positive infinity.
   *
   * @return upper - lower
   */
  constexpr double Range() const noexcept { return upper - lower; }

  /**
   * Check if this scalar bound only contains one point. This only
   * occurs when lower == upper.
   *
   * @return lower == upper
   */
  constexpr bool IsExact() const noexcept { return lower == upper; }

  /**
   * Check if this scalar bound only contains 0. This occurs when
   * this scalar bound equals 0.0.
   *
   * @return lower == 0.0 && upper == 0.0
   */
  constexpr bool IsZero() const noexcept {
    return lower == 0.0 && upper == 0.0;
  }

  /**
   * Returns true if this IntervalSet1d has a lower bound.
   */
  constexpr bool IsLowerBounded() const noexcept {
    return lower > -std::numeric_limits<double>::infinity();
  }

  /**
   * Returns true if this IntervalSet1d has an upper bound.
   */
  constexpr bool IsUpperBounded() const noexcept {
    return upper < +std::numeric_limits<double>::infinity();
  }

  /**
   * Check if this scalar bound is valid. A scalar bound is valid
   * if and only if the lower bound is less than or equal to the upper
   * bound.
   *
   * @return lower <= upper
   */
  constexpr bool IsValid() const noexcept { return lower <= upper; }
};

}  // namespace trajopt
