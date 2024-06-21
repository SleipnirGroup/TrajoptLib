// Copyright (c) TrajoptLib contributors

#pragma once

#include <cassert>

#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * Elliptical 2D set.
 */
struct TRAJOPT_DLLEXPORT EllipticalSet2d {
  /**
   * The directionality of the set.
   */
  enum class Direction {
    /// The set is every point inside the ellipse.
    kInside,
    /// The set is every point on the border of the ellipse.
    kCentered,
    /// The set is every point outside the ellipse.
    kOutside
  };

  /// The x radius.
  double xRadius;

  /// The y radius.
  double yRadius;

  /// The set direction.
  Direction direction;

  /**
   * Construct an EllipticalSet2d.
   *
   * @param xRadius The ellipse's x radius. Must be greater than zero.
   * @param yRadius The ellipse's y radius. Must be greater than zero.
   * @param direction The set direction.
   */
  constexpr EllipticalSet2d(double xRadius, double yRadius, Direction direction)
      : xRadius{xRadius}, yRadius{yRadius}, direction{direction} {
    assert(xRadius > 0.0 && yRadius > 0.0);
  }

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
};

}  // namespace trajopt
