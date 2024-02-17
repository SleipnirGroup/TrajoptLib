// Copyright (c) TrajoptLib contributors

#include "trajopt/set/Set2d.h"

#include <optional>
#include <variant>

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
