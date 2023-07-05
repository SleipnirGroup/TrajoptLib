// Copyright (c) TrajoptLib contributors

#include "trajopt/set/IntervalSet1d.h"

#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include <nlohmann/json.hpp>

#include "trajopt/solution/SolutionChecking.h"

namespace trajopt {

IntervalSet1d::IntervalSet1d(double lower, double upper)
    : lower(lower), upper(upper) {}

IntervalSet1d::IntervalSet1d(double value) : lower(value), upper(value) {}

IntervalSet1d IntervalSet1d::R1() {
  return IntervalSet1d(-std::numeric_limits<double>::infinity(),
                       +std::numeric_limits<double>::infinity());
}

IntervalSet1d IntervalSet1d::LessThan(double max) {
  return IntervalSet1d(-std::numeric_limits<double>::infinity(), max);
}

IntervalSet1d IntervalSet1d::GreaterThan(double min) {
  return IntervalSet1d(min, +std::numeric_limits<double>::infinity());
}

double IntervalSet1d::Range() const noexcept {
  return upper - lower;
}

bool IntervalSet1d::IsExact() const noexcept {
  return lower == upper;
}

bool IntervalSet1d::IsZero() const noexcept {
  return lower == 0.0 && upper == 0.0;
}

bool IntervalSet1d::IsLowerBounded() const noexcept {
  return lower > -std::numeric_limits<double>::infinity();
}
bool IntervalSet1d::IsUpperBounded() const noexcept {
  return upper < +std::numeric_limits<double>::infinity();
}

std::optional<SolutionError> IntervalSet1d::CheckScalar(
    double scalar, const SolutionTolerances& tolerances) const noexcept {
  if ((IsExact() && std::abs(scalar - lower) > tolerances.errorMargin) ||
      scalar < lower || scalar > upper) {
    return SolutionError{fmt::format("= {}", scalar)};
  }
  return std::nullopt;
}

bool IntervalSet1d::IsValid() const noexcept {
  return lower <= upper;
}

void to_json(nlohmann::json& j, const IntervalSet1d& set1d) {
  if (set1d.IsExact()) {
    j = set1d.lower;
  } else {
    if (set1d.IsLowerBounded()) {
      j = nlohmann::json{{"lower", set1d.lower}};
    } else {
      j.update({"lower", "-inf"});
    }
    if (set1d.IsUpperBounded()) {
      j.push_back({"upper", set1d.upper});
    } else {
      j.push_back({"upper", "+inf"});
    }
  }
}

void from_json(const nlohmann::json& j, IntervalSet1d& set1d) {
  if (j.is_number()) {
    set1d = IntervalSet1d(j.get<double>());
  } else {
    auto lower = j.at("lower");
    if (lower.is_string() && lower.get<std::string>() == "-inf") {
      set1d.lower = -std::numeric_limits<double>::infinity();
    } else if (lower.is_number()) {
      set1d.lower = lower.get<double>();
    }
    auto upper = j.at("upper");
    if (upper.is_string() && upper.get<std::string>() == "+inf") {
      set1d.upper = +std::numeric_limits<double>::infinity();
    } else if (upper.is_number()) {
      set1d.upper = upper.get<double>();
    }
  }
}

}  // namespace trajopt
