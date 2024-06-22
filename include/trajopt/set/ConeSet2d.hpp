// Copyright (c) TrajoptLib contributors

#pragma once

#include <cassert>
#include <numbers>

#include "trajopt/set/IntervalSet1d.hpp"
#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * Conical 2D set.
 */
struct TRAJOPT_DLLEXPORT ConeSet2d {
  /// The heading bounds of the cone.
  IntervalSet1d thetaBound;

  /**
   * Constructs a ConeSet2d.
   *
   * @param thetaBound The internal angle of the cone tip. Must be within (0,
   *   π].
   */
  explicit constexpr ConeSet2d(const IntervalSet1d& thetaBound)
      : thetaBound{thetaBound} {
    assert(thetaBound.Range() > 0.0 && thetaBound.Range() <= std::numbers::pi);
  }
};

}  // namespace trajopt
