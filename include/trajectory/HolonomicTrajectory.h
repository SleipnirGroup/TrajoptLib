// Copyright (c) TrajoptLib contributors

#pragma once

#include <string>
#include <vector>

#include <fmt/format.h>

#include "SymbolExports.h"
#include "trajectory/HolonomicTrajectorySample.h"

namespace trajopt {

class TRAJOPT_DLLEXPORT HolonomicTrajectory {
 public:
  std::vector<HolonomicTrajectorySample> samples;

  explicit HolonomicTrajectory(
      const std::vector<HolonomicTrajectorySample>& samples);
  explicit HolonomicTrajectory(
      std::vector<HolonomicTrajectorySample>&& samples);
};
}  // namespace trajopt

template <>
struct fmt::formatter<trajopt::HolonomicTrajectory> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const trajopt::HolonomicTrajectory& trajectory,
              FormatContext& ctx) {
    std::string sampsStr = fmt::format("{}", trajectory.samples[0]);
    for (size_t i = 1; i < trajectory.samples.size(); i++) {
      sampsStr += fmt::format(", {}", trajectory.samples[i]);
    }
    return fmt::format_to(ctx.out(), "[{}]", sampsStr);
  }
};
