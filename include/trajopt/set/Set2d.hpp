// Copyright (c) TrajoptLib contributors

#pragma once

#include <variant>

#include "trajopt/set/ConeSet2d.hpp"
#include "trajopt/set/EllipticalSet2d.hpp"
#include "trajopt/set/LinearSet2d.hpp"
#include "trajopt/set/ManifoldIntervalSet2d.hpp"
#include "trajopt/set/RectangularSet2d.hpp"

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
using Set2d = std::variant<RectangularSet2d, LinearSet2d, EllipticalSet2d,
                           ConeSet2d, ManifoldIntervalSet2d>;

}  // namespace trajopt
