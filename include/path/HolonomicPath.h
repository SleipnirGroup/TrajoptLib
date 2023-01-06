// Copyright (c) TrajoptLib contributors

#pragma once

#include <iostream>
#include <vector>

#include "SymbolExports.h"
#include "path/HolonomicWaypoint.h"
#include "path/Path.h"

namespace trajopt {

/**
 * @brief A sequence of holonomic waypoints that make up a path that a holonomic
 * drivetrain robot can follow. Note that, unlike a path, which is only a
 * general idea of where the robot should go, a Trajectory is the detailed
 * output of the generator that tells the robot exactly how to move.
 */
class TRAJOPT_DLLEXPORT HolonomicPath : public Path {
 public:
  /**
   * @brief the holonomic waypoints that make up this path
   */
  std::vector<HolonomicWaypoint> holonomicWaypoints;

  std::vector<HolonomicConstraint> globalHolonomicConstraints;

  /**
   * @brief Construct a Holonomic Path with a list of holonomic waypoints
   *
   * @param waypoints the holonomic waypoints that make up this path
   */
  HolonomicPath(
      std::vector<HolonomicWaypoint> holonomicWaypoints,
      const Obstacle& bumpers, std::vector<Constraint> globalConstraints = {},
      std::vector<HolonomicConstraint> globalHolonomicConstraints = {});

  /**
   * @brief Get the number of holonomic waypoints that
   * make up this holonomic path.
   *
   * @return the length of this holonomic path
   */
  size_t Length() const noexcept override;
  /**
   * @brief Get the holonomic waypoint at the specified index.
   *
   * @param index the index
   * @return a reference to the holonomic waypoint
   */
  Waypoint& GetWaypoint(size_t index) override;
  /**
   * @brief Get the holonomic waypoint at the specified index.
   *
   * @param index the index
   * @return a const reference to the holonomic waypoint
   */
  const Waypoint& GetWaypoint(size_t index) const override;

  /**
   * @brief Append a string representation of a holonomic path to
   * an output stream. A string representation of a holonomic path
   * is a json array of its holonomic waypoints' json representations.
   *
   * @param stream the stream to append the string representation to
   * @param path the holonomic path
   * @return a reference to the given stream
   */
  friend std::ostream& operator<<(std::ostream& stream,
                                  const HolonomicPath& path);
};
}  // namespace trajopt

template <>
struct fmt::formatter<trajopt::HolonomicPath> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const trajopt::HolonomicPath& holonomicPath, FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "Holonomic Path");
  }
};
