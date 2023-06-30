// Copyright (c) TrajoptLib contributors

#pragma once

#include <string>
#include <vector>

#include <fmt/format.h>

#include "trajopt/SymbolExports.h"
#include "trajopt/solution/HolonomicSolution.h"
#include "trajopt/trajectory/HolonomicTrajectorySample.h"

namespace trajopt {

/**
 * Holonomic trajectory.
 */
class TRAJOPT_DLLEXPORT HolonomicTrajectory {
 public:
  /// Trajectory samples.
  std::vector<HolonomicTrajectorySample> samples;

  /**
   * Construct a HolonomicTrajectory from samples.
   *
   * @param samples The samples.
   */
  explicit HolonomicTrajectory(std::vector<HolonomicTrajectorySample> samples);

  /**
   * Construct a HolonomicTrajectory from a solution.
   *
   * @param solution The solution.
   */
  explicit HolonomicTrajectory(const HolonomicSolution& solution);
};

}  // namespace trajopt

/**
 * Formatter for HolonomicTrajectory.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::HolonomicTrajectory> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted HolonomicTrajectory.
   *
   * @param trajectory HolonomicTrajectory instance.
   * @param ctx Format string context.
   */
  auto format(const trajopt::HolonomicTrajectory& trajectory,
              fmt::format_context& ctx) const {
    std::string sampsStr = fmt::format("{}", trajectory.samples[0]);
    for (size_t i = 1; i < trajectory.samples.size(); i++) {
      sampsStr += fmt::format(", {}", trajectory.samples[i]);
    }
    return fmt::format_to(ctx.out(), "[{}]", sampsStr);
  }
};
