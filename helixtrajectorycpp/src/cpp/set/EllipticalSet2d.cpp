#include "set/EllipticalSet2d.h"

#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include <fmt/format.h>

#include "solution/SolutionChecking.h"

namespace helixtrajectory {

EllipticalSet2d::EllipticalSet2d(double xRadius, double yRadius, Direction direction)
        : xRadius(xRadius), yRadius(yRadius), direction(direction) {
}

EllipticalSet2d EllipticalSet2d::CircularSet2d(double radius, Direction direction) {
    return EllipticalSet2d(radius, radius, direction);
}

bool EllipticalSet2d::IsCircular() const noexcept {
    return xRadius == yRadius;
}

bool EllipticalSet2d::IsR2() const noexcept {
    return xRadius >= std::numeric_limits<double>::infinity()
        && yRadius >= std::numeric_limits<double>::infinity();
}

std::optional<SolutionError> EllipticalSet2d::CheckVector(
        double xComp, double yComp,
        const SolutionTolerances& tolerances) const noexcept {
    auto scaledVectorXSquared = (xComp * xComp) / (xRadius * xRadius);
    auto scaledVectorYSquared = (yComp * yComp) / (yRadius * yRadius);
    auto lhs = scaledVectorXSquared + scaledVectorYSquared;
    switch (direction) {
        case Direction::kInside:
            if (lhs <= 1.0) {
                return SolutionError(fmt::format(
                        "({}, {}) is not on or inside an ellipse with x radius of {} and y radius of {}",
                        xComp, yComp, xRadius, yRadius));
            }
            break;
        case Direction::kCentered:
            if (std::abs(lhs - 1.0) <= tolerances.errorMargin) {
                return SolutionError(fmt::format(
                        "({}, {}) is not on an ellipse with x radius of {} and y radius of {}",
                        xComp, yComp, xRadius, yRadius));
            }
            break;
        case Direction::kOutside:
            if (lhs >= 1.0) {
                return SolutionError(fmt::format(
                        "({}, {}) is not on or outside an ellipse with x radius of {} and y radius of {}",
                        xComp, yComp, xRadius, yRadius));
            }
            break;
    }
    return std::nullopt;
}

bool EllipticalSet2d::IsValid() const noexcept {
    return xRadius > 0.0 && yRadius > 0.0;
}
}

template<typename ParseContext>
constexpr auto fmt::formatter<helixtrajectory::EllipticalSet2d>::parse(
        ParseContext& ctx) {
    return ctx.begin();
}

template<typename FormatContext>
auto fmt::formatter<helixtrajectory::EllipticalSet2d>::format(
        const helixtrajectory::EllipticalSet2d& ellipticalSet,
        FormatContext& ctx) {
    std::string shape;
    if (ellipticalSet.IsCircular()) {
        shape = "circle";
    } else {
        shape = "ellipse";
    }
    using enum helixtrajectory::EllipticalSet2d::Direction;
    std::string direction;
    switch (ellipticalSet.direction) {
        case kInside:
            direction = "inside";
            break;
        case kCentered:
            direction = "centered";
            break;
        case kOutside:
            direction = "outside";
            break;
    }
    return fmt::format_to(ctx.out(), "{}: {}, rₓ = {}, rᵧ = {}",
            shape, direction, ellipticalSet.xRadius, ellipticalSet.yRadius);
}