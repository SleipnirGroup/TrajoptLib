// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include "set/IntervalSet1d.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

class ConeSet2d {
 public:
  IntervalSet1d thetaBound;

  explicit ConeSet2d(const IntervalSet1d& thetaBound);

  std::optional<SolutionError> CheckVector(
      double xComp, double yComp,
      const SolutionTolerances& tolerances) const noexcept;

  bool IsValid() const noexcept;
};
}  // namespace helixtrajectory

template <>
struct fmt::formatter<helixtrajectory::ConeSet2d> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const helixtrajectory::ConeSet2d& coneSet, FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "cone: θ = {}", coneSet.thetaBound);
  }
};
