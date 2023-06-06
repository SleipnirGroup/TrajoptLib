// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include <fmt/core.h>

#include "SymbolExports.h"
#include "drivetrain/SwerveModule.h"
#include "obstacle/Obstacle.h"
#include "trajectory/HolonomicTrajectory.h"

namespace trajopt {

/**
 * @brief This class represents a swerve drivetrain robot. It includes the
 * physical properties necessary to accurately model the dynamics of the system.
 * An arbitrary number of swerve modules can be specified, but typically it will
 * be four. The order the swerve modules are listed does not matter.
 */
struct TRAJOPT_DLLEXPORT SwerveDrivetrain {
  /// the mass of the robot
  double mass;
  /// the moment of inertial of the robot about the origin
  double moi;
  /// The list of swerve modules that make the robot move, usually one in each
  /// corner.
  std::vector<SwerveModule> modules;
};

}  // namespace trajopt

/**
 * Formatter for SwerveDrivetrain.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::SwerveDrivetrain> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted SwerveDrivetrain.
   *
   * @tparam FormatContext Format string context type.
   * @param swerveDrivetrain SwerveDrivetrain instance.
   * @param ctx Format string context.
   */
  auto format(const trajopt::SwerveDrivetrain& swerveDrivetrain,
              fmt::format_context& ctx) {
    return fmt::format_to(ctx.out(),
                          "swerve drivetrain:\n"
                          "  mass = {},\n"
                          "  moi = {},\n"
                          "  modules = (no impl yet)",
                          swerveDrivetrain.mass, swerveDrivetrain.moi);
  }
};
