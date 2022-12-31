#include "constraint/Constraint.h"

#include <optional>
#include <variant>

#include <fmt/format.h>

#include "constraint/HeadingConstraint.h"
#include "constraint/ObstacleConstraint.h"
#include "constraint/PoseConstraint.h"
#include "constraint/TranslationConstraint.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

std::optional<SolutionError> Constraint::CheckState(double x, double y,
        double heading, const SolutionTolerances& tolerances) const noexcept {
    if (IsTranslationConstraint()) {
        std::optional<SolutionError> check = GetTranslationConstraint()
                .CheckTranslation(x, y, tolerances);
        if (check.has_value()) {
            return SolutionError{fmt::format("({}) violated: {}",
                    "GetTranslationConstraint()", check->errorMessage)}; // <<< causes error: "Cannot format a const argument."
        }
    } else if (IsHeadingConstraint()) {
        std::optional<SolutionError> check = GetHeadingConstraint()
                .CheckHeading(heading, tolerances);
        if (check.has_value()) {
            return SolutionError{fmt::format("({}) violated: {}",
                    "GetHeadingConstraint()", check->errorMessage)};
        }
    } else if (IsPoseConstraint()) {
        std::optional<SolutionError> check = GetPoseConstraint()
                .CheckPose(x, y, heading, tolerances);
        if (check.has_value()) {
            return SolutionError{fmt::format("({}) violated: {}",
                    "GetPoseConstraint()", check->errorMessage)};
        }
    }
    return std::nullopt;
}

bool Constraint::IsTranslationConstraint() const {
    return std::holds_alternative<TranslationConstraint>(constraint);
}
bool Constraint::IsHeadingConstraint() const {
    return std::holds_alternative<HeadingConstraint>(constraint);
}
bool Constraint::IsPoseConstraint() const {
    return std::holds_alternative<PoseConstraint>(constraint);
}
bool Constraint::IsObstacleConstraint() const {
    return std::holds_alternative<ObstacleConstraint>(constraint);
}

const TranslationConstraint& Constraint::GetTranslationConstraint() const {
    return std::get<TranslationConstraint>(constraint);
}
TranslationConstraint& Constraint::GetTranslationConstraint() {
    return std::get<TranslationConstraint>(constraint);
}

const HeadingConstraint& Constraint::GetHeadingConstraint() const {
    return std::get<HeadingConstraint>(constraint);
}
HeadingConstraint& Constraint::GetHeadingConstraint() {
    return std::get<HeadingConstraint>(constraint);
}

const PoseConstraint& Constraint::GetPoseConstraint() const {
    return std::get<PoseConstraint>(constraint);
}
PoseConstraint& Constraint::GetPoseConstraint() {
    return std::get<PoseConstraint>(constraint);
}

const ObstacleConstraint& Constraint::GetObstacleConstraint() const {
    return std::get<ObstacleConstraint>(constraint);
}
ObstacleConstraint& Constraint::GetObstacleConstraint() {
    return std::get<ObstacleConstraint>(constraint);
}

Constraint::Constraint(const TranslationConstraint& translationConstraint)
        : constraint(translationConstraint) {
}
Constraint::Constraint(const HeadingConstraint& headingConstraint)
        : constraint(headingConstraint) {
}
Constraint::Constraint(const PoseConstraint& poseConstraint)
        : constraint(poseConstraint) {
}
Constraint::Constraint(const ObstacleConstraint& obstacleConstraint)
        : constraint(obstacleConstraint) {
}
}

template<typename ParseContext>
constexpr auto fmt::formatter<helixtrajectory::Constraint>::parse(
        ParseContext& ctx) {
    return ctx.begin();
}

template<typename FormatContext>
auto fmt::formatter<helixtrajectory::Constraint>::format(
        const helixtrajectory::Constraint& constraint,
        FormatContext& ctx) {
    using namespace helixtrajectory;
    if (constraint.IsTranslationConstraint()) {
        return fmt::format_to(ctx.out(), "constraint: {}", constraint.GetTranslationConstraint());
    } else if (constraint.IsHeadingConstraint()) {
        return fmt::format_to(ctx.out(), "constraint: {}", constraint.GetHeadingConstraint());
    } else if (constraint.IsPoseConstraint()) {
        return fmt::format_to(ctx.out(), "constraint: {}", constraint.GetPoseConstraint());
    } else /*if (constraint.IsObstacleConstraint())*/ {
        return fmt::format_to(ctx.out(), "constraint: {}", constraint.GetObstacleConstraint());
    }
}