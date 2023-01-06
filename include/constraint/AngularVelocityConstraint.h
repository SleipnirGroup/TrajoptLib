// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <fmt/format.h>

#include "SymbolExports.h"
#include "set/IntervalSet1d.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

class TRAJOPT_DLLEXPORT AngularVelocityConstraint {
 public:
  IntervalSet1d angularVelocityBound;

  explicit AngularVelocityConstraint(const IntervalSet1d& angularVelocityBound);

  std::optional<SolutionError> CheckAngularVelocity(
      double angularVelocity,
      const SolutionTolerances& tolerances) const noexcept;
};
}  // namespace trajopt

template <>
struct fmt::formatter<trajopt::AngularVelocityConstraint> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(
      const trajopt::AngularVelocityConstraint& angularVelocityConstraint,
      FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "ω {}",
                          angularVelocityConstraint.angularVelocityBound);
  }
};
