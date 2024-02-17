// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>
#include <string>

#include "trajopt/SymbolExports.h"
#include "trajopt/solution/SolutionChecking.h"

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
   * Returns an error if the given coordinate is outside the ellipse.
   *
   * @param xComp The x coordinate.
   * @param yComp The y coordinate.
   * @param tolerances The tolerances considered to satisfy the constraint.
   */
  std::optional<SolutionError> CheckVector(
      double xComp, double yComp,
      const SolutionTolerances& tolerances) const noexcept;

  /**
   * Returns true if the set is valid.
   */
  bool IsValid() const noexcept;
};

}  // namespace trajopt
