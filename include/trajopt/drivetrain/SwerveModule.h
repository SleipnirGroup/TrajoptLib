// Copyright (c) TrajoptLib contributors

#pragma once

#include <array>

#include "trajopt/SymbolExports.h"
#include "trajopt/constraint/Constraint.h"

namespace trajopt {

/**
 * @brief This class represents a single swerve module in a swerve drivetrain.
 * It is defined by the module diagonal, which is the line connecting the origin
 * of the robot coordinate system to the center of the module. The wheel radius,
 * max speed, and max torque must also be specified per module.
 */
struct TRAJOPT_DLLEXPORT SwerveModule {
  /**
   * @brief x-coordinate of swerve module relative to robot coordinate system
   */
  double x;
  /**
   * @brief y-coordinate of swerve module relative to robot coordinate system
   */
  double y;
  /**
   * @brief radius of wheel
   */
  double wheelRadius;
  /**
   * @brief maximum angular velocity of wheel
   */
  double wheelMaxAngularVelocity;
  /**
   * @brief maximum torque applied to wheel
   */
  double wheelMaxTorque;
};

}  // namespace trajopt

/**
 * Formatter for SwerveModule.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::SwerveModule> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted SwerveModule.
   *
   * @param swerveModule SwerveModule instance.
   * @param ctx Format string context.
   */
  auto format(const trajopt::SwerveModule& swerveModule,
              fmt::format_context& ctx) const {
    return fmt::format_to(
        ctx.out(),
        "swerve module: (x, y) = ({}, {}), r = {}, ωₘₐₓ = {}, τₘₐₓ = {}",
        swerveModule.x, swerveModule.y, swerveModule.wheelRadius,
        swerveModule.wheelMaxAngularVelocity, swerveModule.wheelMaxTorque);
  }
};
