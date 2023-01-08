// Copyright (c) TrajoptLib contributors

#pragma once

#include <cmath>
#include <compare>

namespace trajopt {

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
  double value = 0.0;

  constexpr LooseDouble() = default;
  explicit constexpr LooseDouble(double value) : value(value) {}

  explicit constexpr operator double() { return value; }

  constexpr bool operator==(const LooseDouble& other) const {
    return WithinPrecision(value, other.value, 1e-3);
  }
  auto operator<=>(const LooseDouble& other) const = default;

  LooseDouble operator*(const LooseDouble& other) const {
    return LooseDouble{value * other.value};
  }
};
}  // namespace trajopt
