// Copyright (c) TrajoptLib contributors

#pragma once

#include <cassert>

#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * Manifold 2D set for angles represented as cos-sin pairs.
 */
struct TRAJOPT_DLLEXPORT ManifoldIntervalSet2d {
  /// The angle at the midpoint of the range.
  double middle;

  /// Half of the width of the valid range.
  double tolerance;

  /**
   * Construct a manifold vector bound to a particular value +- a tolerance.
   *
   * @param middle The angle at the midpoint of the valid range
   * @param tolerance The tolerance. Must be positive. Tolerances greater than
   * pi, (half the period) do not constrain the vector at all.
   */
  constexpr ManifoldIntervalSet2d(double middle, double tolerance)
      : middle(middle), tolerance(tolerance) {
    assert(tolerance >= 0);
  }

  /**
   * Construct a manifold vector bound that represents the given angle exactly.
   *
   * @param angle the angle of the vector to.
   */
  constexpr ManifoldIntervalSet2d(double angle)  // NOLINT
      : ManifoldIntervalSet2d(angle, 0) {}

  constexpr ManifoldIntervalSet2d() = default;
};

}  // namespace trajopt
