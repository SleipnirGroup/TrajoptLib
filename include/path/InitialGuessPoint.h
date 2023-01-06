// Copyright (c) TrajoptLib contributors

#pragma once

#include <iostream>

#include <fmt/format.h>

#include "SymbolExports.h"

namespace trajopt {

/**
 * @brief An initial guess of a possible state the robot may be in during the
 * trajectory.
 */
class TRAJOPT_DLLEXPORT InitialGuessPoint {
 public:
  /**
   * @brief the initial guess of the x-coordinate of the robot
   */
  double x;
  /**
   * @brief the initial guess of the y-coordinate of the robot
   */
  double y;
  /**
   * @brief the initial guess of the heading of the robot
   */
  double heading;

  InitialGuessPoint(double x, double y, double heading);

  friend std::ostream& operator<<(std::ostream& stream,
                                  const InitialGuessPoint& guessPoint);
};
}  // namespace trajopt

template <>
struct fmt::formatter<trajopt::InitialGuessPoint> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const trajopt::InitialGuessPoint& initialGuessPoint,
              FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "Initial Guess Point");
  }
};
