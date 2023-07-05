// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <nlohmann/json.hpp>

#include "trajopt/SymbolExports.h"
#include "trajopt/set/IntervalSet1d.h"
#include "trajopt/set/RectangularSet2d.h"
#include "trajopt/solution/SolutionChecking.h"
#include "trajopt/util/JsonFmtFormatter.h"


namespace trajopt {

/**
 * Linear 2D set.
 */
struct TRAJOPT_DLLEXPORT LinearSet2d {
  /// FIXME What does this do?
  double theta;

  /**
   * FIXME What does this do?
   *
   * @param xComp The x coordinate.
   * @param yComp The y coordinate.
   * @param tolerances The tolerances considered to satisfy the constraint.
   */
  std::optional<SolutionError> CheckVector(
      double xComp, double yComp,
      const SolutionTolerances& tolerances) const noexcept;

  /**
   * FIXME What does this do?
   *
   * @param theta FIXME What does this do?
   * @param rBound FIXME What does this do?
   */
  static RectangularSet2d RBoundToRectangular(double theta,
                                              const IntervalSet1d& rBound);
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    LinearSet2d,
    theta)

}  // namespace trajopt

_JSON_FMT_FORMATTER(trajopt::LinearSet2d)
