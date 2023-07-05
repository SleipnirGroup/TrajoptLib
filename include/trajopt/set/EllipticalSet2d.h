// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>
#include <string>

#include <nlohmann/json.hpp>

#include <fmt/format.h>

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

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    EllipticalSet2d,
    xRadius,
    yRadius,
    direction)

}  // namespace trajopt

/**
 * Formatter for EllipticalSet2d.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::EllipticalSet2d> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted EllipticalSet2d.
   *
   * @param ellipticalSet EllipticalSet2d instance.
   * @param ctx Format string context.
   */
  auto format(const trajopt::EllipticalSet2d& ellipticalSet,
              fmt::format_context& ctx) const {
    std::string shape;
    if (ellipticalSet.IsCircular()) {
      shape = "circle";
    } else {
      shape = "ellipse";
    }
    using enum trajopt::EllipticalSet2d::Direction;
    std::string direction;
    switch (ellipticalSet.direction) {
      case kInside:
        direction = "inside";
        break;
      case kCentered:
        direction = "centered";
        break;
      case kOutside:
        direction = "outside";
        break;
    }
    return fmt::format_to(ctx.out(), "{}: {}, rₓ = {}, rᵧ = {}", shape,
                          direction, ellipticalSet.xRadius,
                          ellipticalSet.yRadius);
  }
};
