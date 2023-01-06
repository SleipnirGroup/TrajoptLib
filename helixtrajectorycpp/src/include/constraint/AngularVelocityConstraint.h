// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <fmt/format.h>

#include "set/IntervalSet1d.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

class AngularVelocityConstraint {
 public:
  IntervalSet1d angularVelocityBound;

  explicit AngularVelocityConstraint(const IntervalSet1d& angularVelocityBound);

  std::optional<SolutionError> CheckAngularVelocity(
      double angularVelocity,
      const SolutionTolerances& tolerances) const noexcept;
};
}  // namespace helixtrajectory

template <>
struct fmt::formatter<helixtrajectory::AngularVelocityConstraint> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const helixtrajectory::AngularVelocityConstraint&
                  angularVelocityConstraint,
              FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "ω {}",
                          angularVelocityConstraint.angularVelocityBound);
  }
};
