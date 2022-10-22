#pragma once

#include <limits>

#include "ScalarBound.h"

namespace helixtrajectory {

    /**
     * @brief This class represents a bounded region of abstract 2D space. The bounds
     * are specified with inequalities for each coordinate, a0 and a1. The interpretation of the
     * abstract coordinates a0 and a1 changes when coordinateType changes. If coordinateType
     * is kRectangular, then a0 and a1 represent the rectangular x-coordinate and y-coordinate
     * bounds, respectively. If coordinateType is kPolar, then a0 and a1 represent the polar
     * r-coordinate and theta-coordinate bounds, respectively. Polar theta coordinates
     * must be specified between -pi and pi, inclusive, and all lower bounds must be
     * less than or equal to their respective upper bounds.
     * 
     * The following are some common use cases:
     * A straight line theta = theta_0 bound can be created as a polar bound bounding theta
     * within [theta_0, theta_0], and a straight line x = x_0 or y = y_0 bound can be created by bounding
     * x within [x_0, x_0] or y within [y_0, y_0]. Bounding the magnitude of a vector can be achieved by
     * bounding r within [r_lower, r_upper].
     * 
     * Bounding theta within [-pi, -pi] or within [pi, pi] is equivalent; both will force the vector to
     * be colinear with the x-axis.
     * 
     * @author Justin Babilino
     */
    class PlanarBound {
    public:
        /**
         * @brief Enumerates the supported coordinate system types for planar bounds.
         * This includes rectangular (Cartesian, x-y) and polar coordinates (r-theta).
         */
        enum class CoordinateType {
            /**
             * @brief indicates rectangular coordinates in planar bounds
             */
            kRectangular,
            /**
             * @brief indicates polar coordinates in planar bounds
             */
            kPolar
        };

        /**
         * @brief the first abstract coordinate of the planar bound, indicating either
         * the rectangular x-coordinate bound or the polar r-coordinate bound
         */
        ScalarBound a0;
        /**
         * @brief the bounds on the second abstract coordinate of the planar bound,
         * indicating either the rectangular y-coordinate bound or the polar
         * theta-coordinate bound
         */
        ScalarBound a1;
        /**
         * @brief the coordinate type to use when interpreting a0 and a1
         */
        CoordinateType coordinateType;

        /**
         * @brief Construct a Planar Bound with the boundaries of abstract coordinates
         * a0 and a1 and the type of coordinates. By default, an infinite bound is created.
         * 
         * @param a0 the bounds on the first abstract coordinate
         * @param a1 the bounds on the second abstract coordinate
         * @param coordinateType the coordinate type
         */
        PlanarBound(const ScalarBound& a0 = ScalarBound(), const ScalarBound& a1 = ScalarBound(),
                CoordinateType coordinateType = CoordinateType::kRectangular);

        /**
         * @brief Check if this bound is rectangular.
         * 
         * @return coordinateType == CoordinateType::kRectangular
         */
        bool IsRectangular() const noexcept;
        /**
         * @brief Check if this bound is polar.
         * 
         * @return coordinateType == CoordinateType::kPolar
         */
        bool IsPolar() const noexcept;

        /**
         * @brief Check if this planar bound is circularly symmetric. A planar bound is
         * circularly symmetric if applying a rotation transformation about the origin to
         * the bound would not change the bound. Therefore, circularly symemetry occurs
         * when the bound is polar and a1 has a range of 2*pi or when this bound is zero.
         * 
         * @return IsPolar() && a1.upper - a1.lower >= 2*pi
         */
        bool IsCircularlySymetric() const noexcept;

        /**
         * @brief Check if this planar bound is contained entirely on the x-axis.
         * This occurs for cartesian bounds when y is bounded within [0, 0], and
         * it occurs for polar bounds when theta is bounded within [-pi, -pi],
         * [0, 0], or [pi, pi].
         * 
         * @return true if and only if the bound is contained entirely on the x-axis
         */
        bool IsOnXAxis() const noexcept;
        
        /**
         * Check if this planar bound is contained entirely on the y-axis.
         * This occurs for rectangular bounds when x is bounded within [0, 0], and
         * it occurs for polar bounds when theta is bounded within [-pi/2, -pi/2]
         * or [pi/2, pi/2].
         * 
         * @return true if and only if the bound is contained entirely on the x-axis
         */
        bool IsOnYAxis() const noexcept;

        /**
         * @brief Check if this planar bound is contained entirely on a straight line
         * containing more than one point.
         * 
         * @return 
         */
        bool IsLinear() const noexcept;

        /**
         * @brief Check if this planar bound only contains one point. This
         * occurs when a0 and a1 are exact.
         * 
         * @return a0.IsExact() && a1.IsExact()
         */
        bool IsExact() const noexcept;

        /**
         * @brief Check if this planar bound only contains the origin. This
         * occurs for rectangular bounds when a0 and a1 are both zero and for
         * polar bounds when a0 is zero.
         * 
         * @return a0.IsZero() && (!IsRectangular() || a1.IsZero())
         */
        bool IsZero() const noexcept;

        /**
         * @brief Check if this planar bound is infinite. This occurs for rectangular
         * bounds when a0 and a1 are infinite and for polar bounds when a0 contains
         * the interval [0, inf].
         */
        bool IsInfinite() const noexcept;

        /**
         * @brief Check if this planar bound is valid. A planar bound is valid when the bounds
         * on a0 and a1 are valid, and additionally for planar bounds, a0 is contained within
         * the interval [0, inf] and a1 is contained within
         * the interval [-pi, pi].
         * 
         * @return true if and only if this planar bound is valid
         */
        bool IsValid() const noexcept;
    };
}