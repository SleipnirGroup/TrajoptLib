// Copyright (c) TrajoptLib contributors

#pragma once

#include <limits>

#include "trajopt/SymbolExports.h"

namespace trajopt {

/**
 * Elliptical 2D set.
 */
struct TRAJOPT_DLLEXPORT EllipticalSet2d {
  /**
   * FIXME What does this do?
   */
  enum class Direction {
    /// FIXME What does this do?
    kInside,
    /// FIXME What does this do?
    kCentered,
    /// FIXME What does this do?
    kOutside
  };

  /// The x radius.
  double xRadius;

  /// The y radius.
  double yRadius;

  /// The direction.
  Direction direction;

  /**
   * Construct a circular EllipticalSet2d from a radius.
   *
   * @param radius The radius.
   * @param direction The direction.
   */
  static constexpr EllipticalSet2d CircularSet2d(
      double radius, Direction direction = Direction::kInside) {
    return EllipticalSet2d{radius, radius, direction};
  }

  /**
   * Returns true if the ellipse is a circle.
   */
  constexpr bool IsCircular() const noexcept { return xRadius == yRadius; }

  /**
   * Returns true if the set spans R².
   */
  constexpr bool IsR2() const noexcept {
    return xRadius >= std::numeric_limits<double>::infinity() &&
           yRadius >= std::numeric_limits<double>::infinity();
  }

  /**
   * Returns true if the set is valid.
   */
  constexpr bool IsValid() const noexcept {
    return xRadius > 0.0 && yRadius > 0.0;
  }
};

}  // namespace trajopt
