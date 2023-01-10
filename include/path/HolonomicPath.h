// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include "SymbolExports.h"
#include "path/HolonomicWaypoint.h"
#include "path/Path.h"

namespace trajopt {

/**
 * A sequence of holonomic waypoints that make up a path that a holonomic
 * drivetrain robot can follow. Note that, unlike a path, which is only a
 * general idea of where the robot should go, a Trajectory is the detailed
 * output of the generator that tells the robot exactly how to move.
 */
class TRAJOPT_DLLEXPORT HolonomicPath : public Path {
 public:
  /// The holonomic waypoints that make up this path.
  std::vector<HolonomicWaypoint> holonomicWaypoints;

  /// The global holonomic constraints.
  std::vector<HolonomicConstraint> globalHolonomicConstraints;

  /**
   * Construct a Holonomic Path with a list of holonomic waypoints
   *
   * @param holonomicWaypoints The holonomic waypoints that make up this path.
   * @param bumpers The bumper obstacle.
   * @param globalConstraints List of global constraints.
   * @param globalHolonomicConstraints List of global holonomic constraints.
   */
  HolonomicPath(
      std::vector<HolonomicWaypoint> holonomicWaypoints,
      const Obstacle& bumpers, std::vector<Constraint> globalConstraints = {},
      std::vector<HolonomicConstraint> globalHolonomicConstraints = {});

  /**
   * Get the number of holonomic waypoints that make up this holonomic path.
   *
   * @return the length of this holonomic path
   */
  size_t Length() const noexcept override;

  /**
   * Get the holonomic waypoint at the specified index.
   *
   * @param index the index
   * @return a reference to the holonomic waypoint
   */
  Waypoint& GetWaypoint(size_t index) override;

  /**
   * Get the holonomic waypoint at the specified index.
   *
   * @param index the index
   * @return a const reference to the holonomic waypoint
   */
  const Waypoint& GetWaypoint(size_t index) const override;
};

}  // namespace trajopt

/**
 * Formatter for HolonomicPath.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::HolonomicPath> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted HolonomicPath.
   *
   * @tparam FormatContext Format string context type.
   * @param holonomicPath HolonomicPath instance.
   * @param ctx Format string context.
   */
  template <typename FormatContext>
  auto format(const trajopt::HolonomicPath& holonomicPath, FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "Holonomic Path");
  }
};
