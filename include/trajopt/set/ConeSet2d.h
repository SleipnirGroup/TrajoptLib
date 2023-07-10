// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <nlohmann/json.hpp>

#include "trajopt/SymbolExports.h"
#include "trajopt/set/IntervalSet1d.h"
#include "trajopt/solution/SolutionChecking.h"
#include "trajopt/util/JsonFmtFormatter.h"

namespace trajopt {

/**
 * Conical 2D set.
 */
struct TRAJOPT_DLLEXPORT ConeSet2d {
  /// The heading bounds of the cone.
  IntervalSet1d thetaBound;

  /**
   * Returns an error if the given coordinate is outside the cone.
   *
   * @param xComp The x coordinate.
   * @param yComp The y coordinate.
   * @param tolerances The tolerances considered to satisfy the constraint.
   */
  std::optional<SolutionError> CheckVector(
      double xComp, double yComp,
      const SolutionTolerances& tolerances) const noexcept;

  /**
   * Returns true if the set is valid.
   */
  bool IsValid() const noexcept;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ConeSet2d, thetaBound)

}  // namespace trajopt

_JSON_FMT_FORMATTER(trajopt::ConeSet2d)
