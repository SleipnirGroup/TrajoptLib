// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>
#include <variant>

#include "SymbolExports.h"
#include "set/ConeSet2d.h"
#include "set/EllipticalSet2d.h"
#include "set/LinearSet2d.h"
#include "set/RectangularSet2d.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

/**
 * @brief This class represents a bounded region of abstract 2D space. The
 * bounds are specified with inequalities for each coordinate, a0 and a1. The
 * interpretation of the abstract coordinates a0 and a1 changes when
 * coordinateType changes. If coordinateType is kRectangular, then a0 and a1
 * represent the rectangular x-coordinate and y-coordinate bounds, respectively.
 * If coordinateType is kPolar, then a0 and a1 represent the polar r-coordinate
 * and theta-coordinate bounds, respectively. Polar theta coordinates must be
 * specified between -pi and pi, inclusive, and all lower bounds must be less
 * than or equal to their respective upper bounds.
 *
 * The following are some common use cases:
 * A straight line theta = theta_0 bound can be created as a polar bound
 * bounding theta within [theta_0, theta_0], and a straight line x = x_0 or y =
 * y_0 bound can be created by bounding x within [x_0, x_0] or y within [y_0,
 * y_0]. Bounding the magnitude of a vector can be achieved by bounding r within
 * [r_lower, r_upper].
 *
 * Bounding theta within [-pi, -pi] or within [pi, pi] is equivalent; both will
 * force the vector to be colinear with the x-axis.
 */
using Set2d =
    std::variant<RectangularSet2d, LinearSet2d, EllipticalSet2d, ConeSet2d>;

std::optional<SolutionError> CheckVector(
    const Set2d& set2d,
    double xComp, double yComp, const SolutionTolerances& tolerances);

}  // namespace trajopt

/**
 * Formatter for Set2d.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::Set2d> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted Set2d.
   *
   * @tparam FormatContext Format string context type.
   * @param set2d Set2d instance.
   * @param ctx Format string context.
   */
  template <typename FormatContext>
  auto format(const trajopt::Set2d& set2d, FormatContext& ctx) {
    using namespace trajopt;
    if (std::holds_alternative<RectangularSet2d>(set2d)) {
      return fmt::format_to(ctx.out(), "2d {}", std::get<RectangularSet2d>(set2d));
    } else if (std::holds_alternative<LinearSet2d>(set2d)) {
      return fmt::format_to(ctx.out(), "2d {}", std::get<LinearSet2d>(set2d));
    } else if (std::holds_alternative<EllipticalSet2d>(set2d)) {
      return fmt::format_to(ctx.out(), "2d {}",std::get<EllipticalSet2d>(set2d));
    } else /*if (set2d.IsCone())*/ {
      return fmt::format_to(ctx.out(), "2d {}", std::get<ConeSet2d>(set2d));
    }
  }
};
