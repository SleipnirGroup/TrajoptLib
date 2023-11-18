// Copyright (c) TrajoptLib contributors

#include "trajopt/constraint/holonomic/HolonomicConstraint.h"

#include <optional>
#include <string>
#include <variant>

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include "trajopt/constraint/AngularVelocityConstraint.h"
#include "trajopt/constraint/HeadingConstraint.h"
#include "trajopt/constraint/LinePointConstraint.h"
#include "trajopt/constraint/PointLineConstraint.h"
#include "trajopt/constraint/PointPointConstraint.h"
#include "trajopt/constraint/TranslationConstraint.h"
#include "trajopt/constraint/holonomic/HolonomicVelocityConstraint.h"
#include "trajopt/solution/SolutionChecking.h"

namespace trajopt {

std::optional<SolutionError> CheckState(
    const HolonomicConstraint& constraint, double x, double y, double thetacos, double thetasin,
    double velocityX, double velocityY, double angularVelocity,
    double accelerationX, double accelerationY, double angularAcceleration,
    const SolutionTolerances& tolerances) noexcept {
  if (std::holds_alternative<HolonomicVelocityConstraint>(constraint)) {
    std::optional<SolutionError> check =
        std::get<HolonomicVelocityConstraint>(constraint)
            .CheckVelocity(velocityX, velocityY, tolerances);
    if (check.has_value()) {
      return SolutionError{
          fmt::format("({}) violated: {}", "*this", check->errorMessage)};
    }
  } else if (std::holds_alternative<AngularVelocityConstraint>(constraint)) {
    std::optional<SolutionError> check =
        std::get<AngularVelocityConstraint>(constraint)
            .CheckAngularVelocity(angularVelocity, tolerances);
    if (check.has_value()) {
      return SolutionError{
          fmt::format("({}) violated: {}", "*this", check->errorMessage)};
    }
  }
  return std::nullopt;
}

}  // namespace trajopt

namespace nlohmann {

void adl_serializer<trajopt::HolonomicConstraint>::to_json(
    json& j, const trajopt::HolonomicConstraint& constraint) {
  if (const auto* _constraint =
          std::get_if<trajopt::TranslationConstraint>(&constraint)) {
    j = json{{"constraintType", "translation"}};
    j.update(*_constraint);
  } else if (const auto* _constraint =
                 std::get_if<trajopt::HeadingConstraint>(&constraint)) {
    j = json{{"constraintType", "heading"}};
    j.update(*_constraint);
  } else if (const auto* _constraint =
                 std::get_if<trajopt::LinePointConstraint>(&constraint)) {
    j = json{{"constraintType", "linePoint"}};
    j.update(*_constraint);
  } else if (const auto* _constraint =
                 std::get_if<trajopt::PointLineConstraint>(&constraint)) {
    j = json{{"constraintType", "pointLine"}};
    j.update(*_constraint);
  } else if (const auto* _constraint =
                 std::get_if<trajopt::PointPointConstraint>(&constraint)) {
    j = json{{"constraintType", "pointPoint"}};
    j.update(*_constraint);
  } else if (const auto* _constraint =
                 std::get_if<trajopt::AngularVelocityConstraint>(&constraint)) {
    j = json{{"constraintType", "angularVelocity"}};
    j.update(*_constraint);
  } else if (const auto* _constraint =
                 std::get_if<trajopt::HolonomicVelocityConstraint>(
                     &constraint)) {
    j = json{{"constraintType", "holonomicVelocity"}};
    j.update(*_constraint);
  }
}

void adl_serializer<trajopt::HolonomicConstraint>::from_json(
    const json& j, trajopt::HolonomicConstraint& constraint) {
  std::string type = j.at("constraintType").get<std::string>();
  if (type == "translation") {
    constraint = j.get<trajopt::TranslationConstraint>();
  } else if (type == "heading") {
    constraint = j.get<trajopt::HeadingConstraint>();
  } else if (type == "linePoint") {
    constraint = j.get<trajopt::LinePointConstraint>();
  } else if (type == "pointLine") {
    constraint = j.get<trajopt::PointLineConstraint>();
  } else if (type == "pointPoint") {
    constraint = j.get<trajopt::PointPointConstraint>();
  } else if (type == "angularVelocity") {
    constraint = j.get<trajopt::AngularVelocityConstraint>();
  } else if (type == "holonomicVelocity") {
    constraint = j.get<trajopt::HolonomicVelocityConstraint>();
  }
}

}  // namespace nlohmann
