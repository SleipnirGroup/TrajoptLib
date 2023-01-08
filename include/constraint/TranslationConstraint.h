// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <fmt/format.h>

#include "SymbolExports.h"
#include "set/Set2d.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

class TRAJOPT_DLLEXPORT TranslationConstraint {
 public:
  Set2d translationBound;

  explicit TranslationConstraint(const Set2d& translationBound);

  std::optional<SolutionError> CheckTranslation(
      double x, double y, const SolutionTolerances& tolerances) const noexcept;
};
}  // namespace trajopt

template <>
struct fmt::formatter<trajopt::TranslationConstraint> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const trajopt::TranslationConstraint& translationConstraint,
              FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "translation {}",
                          translationConstraint.translationBound);
  }
};
