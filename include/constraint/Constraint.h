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

using ConstraintVariant = std::variant<TranslationConstraint, HeadingConstraint,
                                       PoseConstraint, ObstacleConstraint>;

/**
 * Constraint.
 */
class TRAJOPT_DLLEXPORT Constraint {
 public:
  /**
   * Returns an error if the state doesn't satisfy the constraint.
   *
   * @param x The x coordinate.
   * @param y The y coordinate.
   * @param heading The heading.
   * @param tolerances The tolerances considered to satisfy the constraint.
   */
  std::optional<SolutionError> CheckState(
      double x, double y, double heading,
      const SolutionTolerances& tolerances) const noexcept;

  /**
   * Returns true if this is a translation constraint.
   */
  bool IsTranslationConstraint() const;

  /**
   * Returns true if this is a heading constraint.
   */
  bool IsHeadingConstraint() const;

  /**
   * Returns true if this is a pose constraint.
   */
  bool IsPoseConstraint() const;

  /**
   * Returns true if this is an obstacle constraint.
   */
  bool IsObstacleConstraint() const;

  /**
   * Returns this constraint as a TranslationConstraint.
   */
  const TranslationConstraint& GetTranslationConstraint() const;

  /**
   * Returns this constraint as a TranslationConstraint.
   */
  TranslationConstraint& GetTranslationConstraint();

  /**
   * Returns this constraint as a heading constraint.
   */
  const HeadingConstraint& GetHeadingConstraint() const;

  /**
   * Returns this constraint as a heading constraint.
   */
  HeadingConstraint& GetHeadingConstraint();

  /**
   * Returns this constraint as a pose constraint.
   */
  const PoseConstraint& GetPoseConstraint() const;

  /**
   * Returns this constraint as a pose constraint.
   */
  PoseConstraint& GetPoseConstraint();

  /**
   * Returns this constraint as an obstacle constraint.
   */
  const ObstacleConstraint& GetObstacleConstraint() const;

  /**
   * Returns this constraint as an obstacle constraint.
   */
  ObstacleConstraint& GetObstacleConstraint();

  /**
   * Constructs this constraint from a TranslationConstraint.
   */
  Constraint(const TranslationConstraint& translationConstraint);  // NOLINT

  /**
   * Constructs this constraint from a HeadingConstraint.
   */
  Constraint(const HeadingConstraint& headingConstraint);  // NOLINT

  /**
   * Constructs this constraint from a PoseConstraint.
   */
  Constraint(const PoseConstraint& poseConstraint);  // NOLINT

  /**
   * Constructs this constraint from a ObstacleConstraint.
   */
  Constraint(const ObstacleConstraint& obstacleConstraint);  // NOLINT

 private:
  ConstraintVariant constraint;
};
}  // namespace trajopt

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
    if (constraint.IsTranslationConstraint()) {
      return fmt::format_to(ctx.out(), "constraint: {}",
                            constraint.GetTranslationConstraint());
    } else if (constraint.IsHeadingConstraint()) {
      return fmt::format_to(ctx.out(), "constraint: {}",
                            constraint.GetHeadingConstraint());
    } else if (constraint.IsPoseConstraint()) {
      return fmt::format_to(ctx.out(), "constraint: {}",
                            constraint.GetPoseConstraint());
    } else if (constraint.IsObstacleConstraint()) {
      return fmt::format_to(ctx.out(), "constraint: {}",
                            constraint.GetObstacleConstraint());
    } else {
      return ctx.out();
    }
  }
};
