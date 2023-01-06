// Copyright (c) TrajoptLib contributors

#pragma once

#include <iostream>

#include <fmt/format.h>

#include "SymbolExports.h"

namespace trajopt {

class TRAJOPT_DLLEXPORT HolonomicTrajectorySample {
 public:
  double timestamp;
  double x;
  double y;
  double heading;
  double velocityX;
  double velocityY;
  double angularVelocity;

  HolonomicTrajectorySample(double timestamp, double x, double y,
                            double heading, double velocityX, double velocityY,
                            double angularVelocity);

  friend std::ostream& operator<<(std::ostream& stream,
                                  const HolonomicTrajectorySample& sample);
};
}  // namespace trajopt

template <>
struct fmt::formatter<trajopt::HolonomicTrajectorySample> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const trajopt::HolonomicTrajectorySample& sample,
              FormatContext& ctx) {
    return fmt::format_to(
        ctx.out(),
        "{{\"timestamp\": {}, \"x\": {}, \"y\": {}, \"heading\": {}, "
        "\"velocityX\": {}, \"velocityY\": {}, \"angularVelocity\": {}}}",
        sample.timestamp, sample.x, sample.y, sample.heading, sample.velocityX,
        sample.velocityY, sample.angularVelocity);
  }
};
