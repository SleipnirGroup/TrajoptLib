// Copyright (c) TrajoptLib contributors

#include "trajopt/set/Set2d.h"

#include <optional>
#include <variant>

#include <nlohmann/json.hpp>

#include "trajopt/set/ConeSet2d.h"
#include "trajopt/set/EllipticalSet2d.h"
#include "trajopt/set/LinearSet2d.h"
#include "trajopt/set/RectangularSet2d.h"
#include "trajopt/solution/SolutionChecking.h"

namespace trajopt {

std::optional<SolutionError> CheckVector(const Set2d& set2d, double xComp,
                                         double yComp,
                                         const SolutionTolerances& tolerances) {
  if (std::holds_alternative<RectangularSet2d>(set2d)) {
    return std::get<RectangularSet2d>(set2d).CheckVector(xComp, yComp,
                                                         tolerances);
  } else if (std::holds_alternative<LinearSet2d>(set2d)) {
    return std::get<LinearSet2d>(set2d).CheckVector(xComp, yComp, tolerances);
  } else if (std::holds_alternative<EllipticalSet2d>(set2d)) {
    return std::get<EllipticalSet2d>(set2d).CheckVector(xComp, yComp,
                                                        tolerances);
  } else /*if (IsCone())*/ {
    return std::get<ConeSet2d>(set2d).CheckVector(xComp, yComp, tolerances);
  }
}

}  // namespace trajopt

namespace nlohmann {

void adl_serializer<trajopt::Set2d>::to_json(
    json& j,
    const trajopt::Set2d& set2d) {

  if (const auto* _set2d =
      std::get_if<trajopt::RectangularSet2d>(&set2d)) {
    j = json{{"set2dType", "rectangular"}};
    j.update(*_set2d);
  } else if (const auto* _set2d =
      std::get_if<trajopt::LinearSet2d>(&set2d)) {
    j = json{{"set2dType", "linear"}};
    j.update(*_set2d);
  } else if (const auto* _set2d =
      std::get_if<trajopt::EllipticalSet2d>(&set2d)) {
    j = json{{"set2dType", "elliptical"}};
    j.update(*_set2d);
  } else if (const auto* _set2d =
      std::get_if<trajopt::ConeSet2d>(&set2d)) {
    j = json{{"set2dType", "cone"}};
    j.update(*_set2d);
  }
}

void adl_serializer<trajopt::Set2d>::from_json(const json& j, trajopt::Set2d& set2d) {
  std::string type = j.at("set2dType").get<std::string>();
  if (type == "rectangular") {
    set2d = j.get<trajopt::RectangularSet2d>();
  } else if (type == "linear") {
    set2d = j.get<trajopt::LinearSet2d>();
  } else if (type == "elliptical") {
    set2d = j.get<trajopt::EllipticalSet2d>();
  } else if (type == "cone") {
    set2d = j.get<trajopt::ConeSet2d>();
  }
}

}
