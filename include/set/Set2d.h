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
using Set2dVariant =
    std::variant<RectangularSet2d, LinearSet2d, EllipticalSet2d, ConeSet2d>;

class TRAJOPT_DLLEXPORT Set2d {
 public:
  std::optional<SolutionError> CheckVector(
      double xComp, double yComp, const SolutionTolerances& tolerances) const;

  bool IsRectangular() const;
  bool IsLinear() const;
  bool IsElliptical() const;
  bool IsCone() const;

  const RectangularSet2d& GetRectangular() const;
  RectangularSet2d& GetRectangular();

  const LinearSet2d& GetLinear() const;
  LinearSet2d& GetLinear();

  const EllipticalSet2d& GetElliptical() const;
  EllipticalSet2d& GetElliptical();

  const ConeSet2d& GetCone() const;
  ConeSet2d& GetCone();

  Set2d(const RectangularSet2d& rectangularSet2d);  // NOLINT
  Set2d(const LinearSet2d& linearSet2d);            // NOLINT
  Set2d(const EllipticalSet2d& ellipticalSet2d);    // NOLINT
  Set2d(const ConeSet2d& coneSet2d);                // NOLINT

 private:
  Set2dVariant set2d;
};
}  // namespace trajopt

template <>
struct fmt::formatter<trajopt::Set2d> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const trajopt::Set2d& set2d, FormatContext& ctx) {
    using namespace trajopt;
    if (set2d.IsRectangular()) {
      return fmt::format_to(ctx.out(), "2d {}", set2d.GetRectangular());
    } else if (set2d.IsLinear()) {
      return fmt::format_to(ctx.out(), "2d {}", set2d.GetLinear());
    } else if (set2d.IsElliptical()) {
      return fmt::format_to(ctx.out(), "2d {}", set2d.GetElliptical());
    } else /*if (set2d.IsCone())*/ {
      return fmt::format_to(ctx.out(), "2d {}", set2d.GetCone());
    }
  }
};
