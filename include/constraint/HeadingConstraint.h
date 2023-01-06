// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <fmt/format.h>

#include "SymbolExports.h"
#include "set/IntervalSet1d.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

class TRAJOPT_DLLEXPORT HeadingConstraint {
 public:
  IntervalSet1d headingBound;

  explicit HeadingConstraint(const IntervalSet1d& headingBound);

  std::optional<SolutionError> CheckHeading(
      double theta, const SolutionTolerances& tolerances) const noexcept;
};
}  // namespace trajopt

template <>
struct fmt::formatter<trajopt::HeadingConstraint> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const trajopt::HeadingConstraint& headingConstraint,
              FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "heading {}",
                          headingConstraint.headingBound);
  }
};
