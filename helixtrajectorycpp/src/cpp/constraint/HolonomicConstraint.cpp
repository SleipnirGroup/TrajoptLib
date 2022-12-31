#include "constraint/HolonomicConstraint.h"

#include <optional>
#include <variant>

#include <fmt/format.h>

#include "constraint/VelocityConstraint.h"
#include "constraint/AngularVelocityConstraint.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

std::optional<SolutionError> HolonomicConstraint::CheckHolonomicState(double x, double y, double heading,
            double velocityX, double velocityY, double angularVelocity,
            double accelerationX, double accelerationY, double angularAcceleration,
            const SolutionTolerances& tolerances) const noexcept {
    if (IsVelocityConstraint()) {
        std::optional<SolutionError> check = GetVelocityConstraint()
                .CheckVelocity(velocityX, velocityY, tolerances);
        if (check.has_value()) {
            return SolutionError{fmt::format("({}) violated: {}",
                    "*this", check->errorMessage)};
        }
    } else if (IsAngularVelocityConstraint()) {
        std::optional<SolutionError> check = GetAngularVelocityConstraint()
                .CheckAngularVelocity(angularVelocity, tolerances);
        if (check.has_value()) {
            return SolutionError{fmt::format("({}) violated: {}",
                    "*this", check->errorMessage)};
        }
    }
    return std::nullopt;
}

bool HolonomicConstraint::IsVelocityConstraint() const noexcept {
    return std::holds_alternative<VelocityConstraint>(constraint);
}
bool HolonomicConstraint::IsAngularVelocityConstraint() const noexcept {
    return std::holds_alternative<AngularVelocityConstraint>(constraint);
}

const VelocityConstraint& HolonomicConstraint::GetVelocityConstraint() const {
    return std::get<VelocityConstraint>(constraint);
}
VelocityConstraint& HolonomicConstraint::GetVelocityConstraint() {
    return std::get<VelocityConstraint>(constraint);
}

const AngularVelocityConstraint& HolonomicConstraint::GetAngularVelocityConstraint() const {
    return std::get<AngularVelocityConstraint>(constraint);
}
AngularVelocityConstraint& HolonomicConstraint::GetAngularVelocityConstraint() {
    return std::get<AngularVelocityConstraint>(constraint);
}

HolonomicConstraint::HolonomicConstraint(const VelocityConstraint& velocityConstraint)
        : constraint(velocityConstraint) {
}
HolonomicConstraint::HolonomicConstraint(const AngularVelocityConstraint& angularVelocityConstraint)
        : constraint(angularVelocityConstraint) {
}
}

template<typename ParseContext>
constexpr auto fmt::formatter<helixtrajectory::Constraint>::parse(
        ParseContext& ctx) {
    return ctx.begin();
}

template<typename FormatContext>
auto fmt::formatter<helixtrajectory::HolonomicConstraint>::format(
        const helixtrajectory::HolonomicConstraint& constraint,
        FormatContext& ctx) {
    if (constraint.IsVelocityConstraint()) {
        return fmt::format_to(ctx.out(), "constraint: {}", constraint.GetVelocityConstraint());
    } else /*if (constraint.IsHeadingConstraint())*/ {
        return fmt::format_to(ctx.out(), "constraint: {}", constraint.GetAngularVelocityConstraint());
    }
}