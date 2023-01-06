// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <fmt/format.h>

#include "SymbolExports.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

/**
 * @brief This class represents a bounded region of abstract 1D space. The bound
 * is specified by an inclusive inequality between two numbers.
 */
class TRAJOPT_DLLEXPORT IntervalSet1d {
 public:
  /**
   * @brief the lower bound
   */
  double lower;
  /**
   * @brief the upper bound
   */
  double upper;

  /**
   * @brief Construct a Scalar Bound between a lower and upper bound.
   *
   * @param lower the lower bound
   * @param upper the upper bound
   */
  IntervalSet1d(double lower, double upper);
  /**
   * @brief Construct a Scalar Bound that represents the interval
   * [value, value].
   *
   * @param value the value to bound the number between
   */
  IntervalSet1d(double value);  // NOLINT

  static IntervalSet1d R1();

  /**
   * @brief Check if this scalar bound is equivalent to another scalar bound.
   * Two scalar bounds are equivalent if their upper and lower bounds are equal.
   * This operator can also be used to check if this bound is equal to a number,
   * meaning the upper and lower bounds both equal the given number.
   *
   * @param other the other scalar bound
   * @return lower == other.lower && upper == other.upper
   */
  bool operator==(const IntervalSet1d& other)
      const noexcept;  // TODO: this might be =default?

  /**
   * @brief Calculate the range of this scalar bound, which is the difference
   * between the upper and lower bounds. An infinite sclar bound has a range
   * of positive infinity.
   *
   * @return upper - lower
   */
  double Range() const noexcept;

  /**
   * @brief Check if this scalar bound only contains one point. This only
   * occurs when lower == upper.
   *
   * @return lower == upper
   */
  bool IsExact() const noexcept;
  /**
   * @brief Check if this scalar bound only contains 0. This occurs when
   * this scalar bound equals 0.0.
   *
   * @return lower == 0.0 && upper == 0.0
   */
  bool IsZero() const noexcept;

  bool IsLowerBounded() const noexcept;
  bool IsUpperBounded() const noexcept;

  std::optional<SolutionError> CheckScalar(
      double scalar, const SolutionTolerances& tolerances) const noexcept;

  /**
   * @brief Check if this scalar bound is valid. A scalar bound is valid
   * if and only if the lower bound is less than or equal to the upper
   * bound.
   *
   * @return lower <= upper
   */
  bool IsValid() const noexcept;
};
}  // namespace trajopt

template <>
struct fmt::formatter<trajopt::IntervalSet1d> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const trajopt::IntervalSet1d& set1d, FormatContext& ctx) {
    if (set1d.IsExact()) {
      return fmt::format_to(ctx.out(), "= {}", set1d.lower);
    } else {
      return fmt::format_to(ctx.out(), "∈ [{}, {}]", set1d.lower, set1d.upper);
    }
  }
};
