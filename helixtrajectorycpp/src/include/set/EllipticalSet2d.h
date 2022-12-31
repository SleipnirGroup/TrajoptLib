#pragma once

#include <optional>

#include <fmt/format.h>

#include "solution/SolutionChecking.h"

namespace helixtrajectory {

class EllipticalSet2d {
public:
    enum class Direction {
        kInside,
        kCentered,
        kOutside
    };

    double xRadius;
    double yRadius;
    Direction direction;

    EllipticalSet2d(double xRadius, double yRadius, Direction direction = Direction::kInside);
    static EllipticalSet2d CircularSet2d(double radius, Direction direction = Direction::kInside);

    bool IsCircular() const noexcept;
    bool IsR2() const noexcept;

    std::optional<SolutionError> CheckVector(double xComp, double yComp, const SolutionTolerances& tolerances) const noexcept;

    bool IsValid() const noexcept;
};
}

template<>
struct fmt::formatter<helixtrajectory::EllipticalSet2d> {

    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx);

    template<typename FormatContext>
    auto format(const helixtrajectory::EllipticalSet2d& ellipticalSet, FormatContext& ctx);
};