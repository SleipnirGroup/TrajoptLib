// Copyright (c) TrajoptLib contributors

#include "set/Set2d.h"

#include <optional>
#include <variant>

#include "set/ConeSet2d.h"
#include "set/EllipticalSet2d.h"
#include "set/LinearSet2d.h"
#include "set/RectangularSet2d.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

std::optional<SolutionError> CheckVector(
    const Set2d& set2d,
    double xComp, double yComp, const SolutionTolerances& tolerances) {
  if (std::holds_alternative<RectangularSet2d>(set2d)) {
    return std::get<RectangularSet2d>(set2d).CheckVector(xComp, yComp, tolerances);
  } else if (std::holds_alternative<LinearSet2d>(set2d)) {
    return std::get<LinearSet2d>(set2d).CheckVector(xComp, yComp, tolerances);
  } else if (std::holds_alternative<EllipticalSet2d>(set2d)) {
    return std::get<EllipticalSet2d>(set2d).CheckVector(xComp, yComp, tolerances);
  } else /*if (IsCone())*/ {
    return std::get<ConeSet2d>(set2d).CheckVector(xComp, yComp, tolerances);
  }
}
}  // namespace trajopt
