// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <fmt/format.h>

#include "SymbolExports.h"
#include "constraint/HeadingConstraint.h"
#include "constraint/TranslationConstraint.h"
#include "set/IntervalSet1d.h"
#include "set/Set2d.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

class TRAJOPT_DLLEXPORT PoseConstraint : public TranslationConstraint,
                                         public HeadingConstraint {
 public:
  PoseConstraint(const Set2d& translationBound,
                 const IntervalSet1d& headingBound);

  std::optional<SolutionError> CheckPose(
      double x, double y, double heading,
      const SolutionTolerances& tolerances) const noexcept;
};
}  // namespace trajopt

template <>
struct fmt::formatter<trajopt::PoseConstraint> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const trajopt::PoseConstraint& poseConstraint,
              FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "obstacle constraint");
  }
};
