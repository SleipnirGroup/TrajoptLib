#include "constraint/ObstacleConstraint.h"

#include "<fmt/format.h>"

#include "obstacle/Obstacle.h"

namespace helixtrajectory {

ObstacleConstraint::ObstacleConstraint(const Obstacle& obstacle)
        : obstacle(obstacle) {
}
}

template<typename ParseContext>
constexpr auto fmt::formatter<helixtrajectory::ObstacleConstraint>::parse(
        ParseContext& ctx) {
    return ctx.begin();
}

template<typename FormatContext>
auto fmt::formatter<helixtrajectory::ObstacleConstraint>::format(
        const helixtrajectory::ObstacleConstraint& obstacleConstraint,
        FormatContext& ctx) {
    return std::format_to(ctx.out(), "");
    // return fmt::format_to(ctx.out(), "heading {}", obstacleConstraint.obstacle);
}