// Copyright (c) TrajoptLib contributors

#pragma once

#include <fmt/format.h>

#include "obstacle/Obstacle.h"

namespace helixtrajectory {

class ObstacleConstraint {
 public:
  Obstacle obstacle;

  explicit ObstacleConstraint(const Obstacle& obstacle);
};
}  // namespace helixtrajectory

template <>
struct fmt::formatter<helixtrajectory::ObstacleConstraint> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const helixtrajectory::ObstacleConstraint& obstacleConstraint,
              FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "obstacle constraint");
  }
};
