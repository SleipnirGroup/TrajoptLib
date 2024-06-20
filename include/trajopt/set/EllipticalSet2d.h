// Copyright (c) TrajoptLib contributors

#pragma once

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
  static EllipticalSet2d CircularSet2d(
      double radius, Direction direction = Direction::kInside);

  /**
   * Returns true if the ellipse is a circle.
   */
  bool IsCircular() const noexcept;

  /**
   * Returns true if the set spans R².
   */
  bool IsR2() const noexcept;

  /**
   * Returns true if the set is valid.
   */
  bool IsValid() const noexcept;
};

}  // namespace trajopt
