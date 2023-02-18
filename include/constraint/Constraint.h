// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>
#include <variant>

#include <fmt/format.h>

#include "SymbolExports.h"
#include "constraint/HeadingConstraint.h"
#include "constraint/ObstacleConstraint.h"
#include "constraint/PoseConstraint.h"
#include "constraint/TranslationConstraint.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

/**
 * @brief Enumerates the various coordinate systems used in trajectory
 * optimization. The field coordinate system is the base system that is fixed to
 * the field with some arbitrary center. Other coordinate systems defined do not
 * have a relative velocity to the field coordinate system, but their position
 * and orientation depends on the current position of the robot. For example, if
 * the robot is spinning, then the robot has angular velocity relative to all
 * the systems defined here, and the magnitude of the robot's velocity is the
 * same in all the systems but the direction varies.
 *
 * We define these other systems: the robot coordinate system, the
 * robot nonrotating coordinate system, and the robot velocity coordinate
 * system. The robot coordinate system is centered on and oriented with the
 * front of the robot towards the x-axis. The robot nonrotating coordinate
 * system is centered on the robot but is oriented with the field. The robot
 * velocity coordinate system is centered on the robot and it is oriented with
 * the robot's velocity in the x-direction.
 *
 * Note that in differential drivetrains, the robot coordinate system
 * is equivalent ot the robot velocity coordinate system.
 */
enum class CoordinateSystem {
  /**
   * @brief the coordinate system of the field
   */
  kField,
  /**
   * @brief the coordinate system of the robot
   */
  kRobot,
};

using Constraint = std::variant<TranslationConstraint, HeadingConstraint,
                                       PoseConstraint, ObstacleConstraint>;

/**
  * Returns an error if the state doesn't satisfy the constraint.
  *
  * @param x The x coordinate.
  * @param y The y coordinate.
  * @param heading The heading.
  * @param tolerances The tolerances considered to satisfy the constraint.
  */
std::optional<SolutionError> CheckState(
    const Constraint& constraint,
    double x, double y, double heading,
    const SolutionTolerances& tolerances) noexcept;
}

/**
 * Formatter for Constraint.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::Constraint> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted Constraint.
   *
   * @tparam FormatContext Format string context type.
   * @param constraint Constraint instance.
   * @param ctx Format string context.
   */
  template <typename FormatContext>
  auto format(const trajopt::Constraint& constraint, FormatContext& ctx) {
    using namespace trajopt;
    if (std::holds_alternative<TranslationConstraint>(constraint)) {
      return fmt::format_to(ctx.out(), "constraint: {}",
                            std::get<TranslationConstraint>(constraint));
    } else if (std::holds_alternative<HeadingConstraint>(constraint)) {
      return fmt::format_to(ctx.out(), "constraint: {}",
                            std::get<HeadingConstraint>(constraint));
    } else if (std::holds_alternative<PoseConstraint>(constraint)) {
      return fmt::format_to(ctx.out(), "constraint: {}",
                            std::get<PoseConstraint>(constraint));
    } else if (std::holds_alternative<ObstacleConstraint>(constraint)) {
      return fmt::format_to(ctx.out(), "constraint: {}",
                            std::get<ObstacleConstraint>(constraint));
    } else {
      return ctx.out();
    }
  }
};
