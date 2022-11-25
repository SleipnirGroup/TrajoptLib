#pragma once

namespace helixtrajectory {

template<typename T>
constexpr inline bool WithinPrecision(const T& actualVal, const T& expectedVal, const T& precision) {
    return std::abs(actualVal - expectedVal) <= precision;
}
}