// Copyright (c) TrajoptLib contributors

#pragma once

#include <string>
#include <vector>

#include <fmt/format.h>

#include "trajectory/HolonomicTrajectorySample.h"

namespace helixtrajectory {

class HolonomicTrajectory {
 public:
  std::vector<HolonomicTrajectorySample> samples;

  explicit HolonomicTrajectory(
      const std::vector<HolonomicTrajectorySample>& samples);
  explicit HolonomicTrajectory(
      std::vector<HolonomicTrajectorySample>&& samples);
};
}  // namespace helixtrajectory

template <>
struct fmt::formatter<helixtrajectory::HolonomicTrajectory> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const helixtrajectory::HolonomicTrajectory& trajectory,
              FormatContext& ctx) {
    std::string sampsStr = fmt::format("{}", trajectory.samples[0]);
    for (int i = 1; i < trajectory.samples.size(); i++) {
      sampsStr += fmt::format(", {}", trajectory.samples[i]);
    }
    return fmt::format_to(ctx.out(), "[{}]", sampsStr);
  }
};
