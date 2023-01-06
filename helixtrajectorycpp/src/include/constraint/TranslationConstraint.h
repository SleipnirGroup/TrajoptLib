#pragma once

#include <optional>

#include <fmt/format.h>

#include "set/Set2d.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

class TranslationConstraint {
public:
    Set2d translationBound;

    TranslationConstraint(const Set2d& translationBound);

    std::optional<SolutionError> CheckTranslation(double x, double y, const SolutionTolerances& tolerances) const noexcept;
};
}

template<>
struct fmt::formatter<helixtrajectory::TranslationConstraint> {

    constexpr auto parse(fmt::format_parse_context& ctx) {
        return ctx.begin();
    }

    template<typename FormatContext>
    auto format(const helixtrajectory::TranslationConstraint& translationConstraint,
            FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "translation {}", translationConstraint.translationBound);
    }
};