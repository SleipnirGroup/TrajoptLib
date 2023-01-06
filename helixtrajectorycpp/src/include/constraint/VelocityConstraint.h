// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <fmt/format.h>

#include "constraint/Constraint.h"
#include "set/Set2d.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

class VelocityConstraint {
 public:
  Set2d velocityBound;
  CoordinateSystem coordinateSystem;

  VelocityConstraint(
      const Set2d& velocityBound,
      CoordinateSystem coordinateSystem = CoordinateSystem::kField);

  std::optional<SolutionError> CheckVelocity(
      double velocityX, double velocityY,
      const SolutionTolerances& tolerances) const noexcept;
};
}  // namespace helixtrajectory

template <>
struct fmt::formatter<helixtrajectory::VelocityConstraint> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const helixtrajectory::VelocityConstraint& velocityConstraint,
              FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "velocity {}",
                          velocityConstraint.velocityBound);
  }
};
