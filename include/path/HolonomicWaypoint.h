// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include "SymbolExports.h"
#include "constraint/Constraint.h"
#include "constraint/HolonomicConstraint.h"
#include "obstacle/Obstacle.h"
#include "path/InitialGuessPoint.h"
#include "path/Waypoint.h"

namespace trajopt {

/**
 * @brief A waypoint in a holonomic path. This class includes additional
 * velocity constraints specific to holonomic drivetrains.
 */
class TRAJOPT_DLLEXPORT HolonomicWaypoint : public Waypoint {
 public:
  /// Waypoint holonomic constraints.
  std::vector<HolonomicConstraint> waypointHolonomicConstraints;

  /// Segment holonomic constraints.
  std::vector<HolonomicConstraint> segmentHolonomicConstraints;

  /**
   * @brief Construct a new Holonomic Waypoint object with its position and
   * velocity state and constraint options, control interval count, and initial
   * guess points.
   *
   * @param waypointConstraints Waypoint constraints.
   * @param waypointHolonomicConstraints Holonomic waypoint constraints.
   * @param segmentConstraints Segment constraints.
   * @param segmentHolonomicConstraints Segment holonomic constraints.
   * @param controlIntervalCount the number of control intervals in the
   *   optimization problem from the previous waypoint to this waypoint
   * @param initialGuessPoints the points used to construct the linear initial
   *   trajectory guess for the trajectory segment from the last waypoint to
   *   this waypoint
   */
  explicit HolonomicWaypoint(
      std::vector<Constraint> waypointConstraints,
      std::vector<HolonomicConstraint> waypointHolonomicConstraints = {},
      std::vector<Constraint> segmentConstraints = {},
      std::vector<HolonomicConstraint> segmentHolonomicConstraints = {},
      size_t controlIntervalCount = 100,
      std::vector<InitialGuessPoint> initialGuessPoints = {});

  /**
   * @brief Check if the velocity state at this holonomic waypoint is known.
   * This means that the velocity vector and angular velocity of the robot is
   * constrained.
   *
   * @return true if the velocity state of the robot is constrained, false
   * otherwise
   */
  bool IsVelocityStateKnown() const noexcept override;
};

}  // namespace trajopt

/**
 * Formatter for HolonomicWaypoint.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::HolonomicWaypoint> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted HolonomicWaypoint.
   *
   * @tparam FormatContext Format string context type.
   * @param holonomicWaypoint HolonomicWaypoint instance.
   * @param ctx Format string context.
   */
  template <typename FormatContext>
  auto format(const trajopt::HolonomicWaypoint& holonomicWaypoint,
              FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "Holonomic Waypoint");
  }
};
