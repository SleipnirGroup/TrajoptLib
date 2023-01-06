// Copyright (c) TrajoptLib contributors

#pragma once

#include <cmath>
#include <compare>

namespace helixtrajectory {

template <typename T>
constexpr inline bool WithinPrecision(const T& actualVal, const T& expectedVal,
                                      const T& precision) {
  // return std::abs(actualVal - expectedVal) <= precision;
  if (actualVal > expectedVal) {
    return actualVal - expectedVal <= precision;
  } else {
    return expectedVal - actualVal <= precision;
  }
}

struct LooseDouble {
  double value;

  constexpr LooseDouble() : value(0.0) {}
  constexpr LooseDouble(double value) : value(value) {}

  constexpr operator double() { return value; }

  // LooseDouble& operator=(LooseDouble other) {
  //     value = other.value;
  //     return *this;
  // }

  constexpr bool operator==(const LooseDouble& other) const {
    return WithinPrecision(value, other.value, 1e-3);
  }
  auto operator<=>(const LooseDouble& other) const = default;

  LooseDouble operator*(const LooseDouble& other) const {
    return value * other.value;
  }
};

// LooseDouble sin(const LooseDouble& theta) {
//     return std::sin(theta.value);
// }
// LooseDouble cos(const LooseDouble& theta) {
//     return std::cos(theta.value);
// }
}  // namespace helixtrajectory
