// Copyright (c) TrajoptLib contributors

#pragma once

#include <fmt/format.h>

#include "SymbolExports.h"
#include "obstacle/Obstacle.h"

namespace trajopt {

class TRAJOPT_DLLEXPORT ObstacleConstraint {
 public:
  Obstacle obstacle;

  explicit ObstacleConstraint(Obstacle obstacle);
};
}  // namespace trajopt

template <>
struct fmt::formatter<trajopt::ObstacleConstraint> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const trajopt::ObstacleConstraint& obstacleConstraint,
              FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "obstacle constraint");
  }
};
