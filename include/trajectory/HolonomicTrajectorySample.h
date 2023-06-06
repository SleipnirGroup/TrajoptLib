// Copyright (c) TrajoptLib contributors

#pragma once

#include <fmt/core.h>

#include "SymbolExports.h"

namespace trajopt {

/**
 * Holonomic trajectory sample.
 */
class TRAJOPT_DLLEXPORT HolonomicTrajectorySample {
 public:
  /// The timestamp.
  double timestamp;

  /// The x coordinate.
  double x;

  /// The y coordinate.
  double y;

  /// The heading.
  double heading;

  /// The velocity's x component.
  double velocityX;

  /// The velocity's y component.
  double velocityY;

  /// The angular velocity.
  double angularVelocity;

  /**
   * Construct a HolonomicTrajectorySample.
   *
   * @param timestamp The timestamp.
   * @param x The x coordinate.
   * @param y The y coordinate.
   * @param heading The heading.
   * @param velocityX The velocity's x component.
   * @param velocityY The velocity's y component.
   * @param angularVelocity The angular velocity.
   */
  HolonomicTrajectorySample(double timestamp, double x, double y,
                            double heading, double velocityX, double velocityY,
                            double angularVelocity);
};

}  // namespace trajopt

/**
 * Formatter for HolonomicTrajectorySample.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::HolonomicTrajectorySample> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted HolonomicTrajectorySample.
   *
   * @tparam FormatContext Format string context type.
   * @param sample HolonomicTrajectorySample instance.
   * @param ctx Format string context.
   */
  auto format(const trajopt::HolonomicTrajectorySample& sample,
              fmt::format_context& ctx) {
    return fmt::format_to(
        ctx.out(),
        "{{\"timestamp\": {}, \"x\": {}, \"y\": {}, \"heading\": {}, "
        "\"velocityX\": {}, \"velocityY\": {}, \"angularVelocity\": {}}}",
        sample.timestamp, sample.x, sample.y, sample.heading, sample.velocityX,
        sample.velocityY, sample.angularVelocity);
  }
};
