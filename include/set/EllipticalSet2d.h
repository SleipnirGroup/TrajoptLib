// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>
#include <string>

#include <fmt/format.h>

#include "SymbolExports.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

class TRAJOPT_DLLEXPORT EllipticalSet2d {
 public:
  enum class Direction { kInside, kCentered, kOutside };

  double xRadius;
  double yRadius;
  Direction direction;

  EllipticalSet2d(double xRadius, double yRadius,
                  Direction direction = Direction::kInside);
  static EllipticalSet2d CircularSet2d(
      double radius, Direction direction = Direction::kInside);

  bool IsCircular() const noexcept;
  bool IsR2() const noexcept;

  std::optional<SolutionError> CheckVector(
      double xComp, double yComp,
      const SolutionTolerances& tolerances) const noexcept;

  bool IsValid() const noexcept;
};
}  // namespace trajopt

template <>
struct fmt::formatter<trajopt::EllipticalSet2d> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const trajopt::EllipticalSet2d& ellipticalSet,
              FormatContext& ctx) {
    std::string shape;
    if (ellipticalSet.IsCircular()) {
      shape = "circle";
    } else {
      shape = "ellipse";
    }
    using enum trajopt::EllipticalSet2d::Direction;
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
    return fmt::format_to(ctx.out(), "{}: {}, rₓ = {}, rᵧ = {}", shape,
                          direction, ellipticalSet.xRadius,
                          ellipticalSet.yRadius);
  }
};
