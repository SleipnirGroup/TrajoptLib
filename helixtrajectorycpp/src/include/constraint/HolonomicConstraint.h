#pragma once

#include <variant>

#include <fmt/format.h>

#include "constraint/AngularVelocityConstraint.h"
#include "constraint/VelocityConstraint.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

using HolonomicConstraintVariant = std::variant<VelocityConstraint, AngularVelocityConstraint>;

class HolonomicConstraint {
public:
    std::optional<SolutionError> CheckHolonomicState(double x, double y, double heading,
            double velocityX, double velocityY, double angularVelocity,
            double accelerationX, double accelerationY, double angularAcceleration,
            const SolutionTolerances& tolerances) const noexcept;

    bool IsVelocityConstraint() const noexcept;
    bool IsAngularVelocityConstraint() const noexcept;

    const VelocityConstraint& GetVelocityConstraint() const;
    VelocityConstraint& GetVelocityConstraint();

    const AngularVelocityConstraint& GetAngularVelocityConstraint() const;
    AngularVelocityConstraint& GetAngularVelocityConstraint();

    HolonomicConstraint(const VelocityConstraint& translationConstraint);
    HolonomicConstraint(const AngularVelocityConstraint& headingConstraint);

private:
    HolonomicConstraintVariant constraint;
};
}

template<>
struct fmt::formatter<helixtrajectory::HolonomicConstraint> {

    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx);

    template<typename FormatContext>
    auto format(const helixtrajectory::HolonomicConstraint& constraint, FormatContext& ctx);
};