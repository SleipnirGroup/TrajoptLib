#include "constraint/TranslationConstraint.h"

#include <optional>

#include <fmt/format.h>

#include "set/Set2d.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

TranslationConstraint::TranslationConstraint(const Set2d& translationBound)
        : translationBound(translationBound) {
}

std::optional<SolutionError> TranslationConstraint::CheckTranslation(
        double x, double y, const SolutionTolerances& tolerances) const noexcept {
    auto check = translationBound.CheckVector(x, y, tolerances);
    if (check.has_value()) {
        return SolutionError{fmt::format("translation = (x, y) = ({}, {}): x-component: ", x, y, check->errorMessage)};
    }
    return std::nullopt;
}
}

template<typename ParseContext>
constexpr auto fmt::formatter<helixtrajectory::TranslationConstraint>::parse(
        ParseContext& ctx) {
    return ctx.begin();
}

template<typename FormatContext>
auto fmt::formatter<helixtrajectory::TranslationConstraint>::format(
        const helixtrajectory::TranslationConstraint& translationConstraint,
        FormatContext& ctx) {
    return std::format_to(ctx.out(), "");
    // return fmt::format_to(ctx.out(), "heading {}", obstacleConstraint.obstacle);
}