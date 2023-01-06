// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include "set/IntervalSet1d.h"
#include "set/RectangularSet2d.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

class LinearSet2d {
 public:
  double theta;

  explicit LinearSet2d(double theta);

  std::optional<SolutionError> CheckVector(
      double xComp, double yComp,
      const SolutionTolerances& tolerances) const noexcept;

  static RectangularSet2d RBoundToRectangular(double theta,
                                              const IntervalSet1d& rBound);
};
}  // namespace helixtrajectory

template <>
struct fmt::formatter<helixtrajectory::LinearSet2d> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const helixtrajectory::LinearSet2d& linearSet,
              FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "polar line: θ = {}", linearSet.theta);
  }
};
