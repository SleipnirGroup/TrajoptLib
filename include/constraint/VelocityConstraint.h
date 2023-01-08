// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <fmt/format.h>

#include "SymbolExports.h"
#include "constraint/Constraint.h"
#include "set/Set2d.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

class TRAJOPT_DLLEXPORT VelocityConstraint {
 public:
  Set2d velocityBound;
  CoordinateSystem coordinateSystem;

  explicit VelocityConstraint(
      const Set2d& velocityBound,
      CoordinateSystem coordinateSystem = CoordinateSystem::kField);

  std::optional<SolutionError> CheckVelocity(
      double velocityX, double velocityY,
      const SolutionTolerances& tolerances) const noexcept;
};
}  // namespace trajopt

template <>
struct fmt::formatter<trajopt::VelocityConstraint> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const trajopt::VelocityConstraint& velocityConstraint,
              FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "velocity {}",
                          velocityConstraint.velocityBound);
  }
};
