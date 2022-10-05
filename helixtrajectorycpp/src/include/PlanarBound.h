#pragma once

#include "ScalarBound.h"

namespace helixtrajectory {

    /**
     * @brief This class represents a bounded region of abstract 2D space. The bounds
     * are specified with inequalities for each coordinate, a0 and a1. The interpretation of the
     * abstract coordinates a0 and a1 change when coordinateType changes. If coordinateType
     * is kRectangular, then a0 and a1 represent the rectangular x-coordinate and y-coordinate
     * bounds, respectively. If coordinateType is kPolar, then a0 and a1 represent the polar
     * r-coordinate and theta-coordinate bounds, respectively. Polar theta coordinates
     * must be specified between -pi and pi, inclusive, and all lower bounds must be
     * less than or equal to their respective upper bounds.
     * 
     * Straight line bounds can be created either by only bounding theta within [theta, theta],
     * x within [x, x], or y within [y, y]. Bounding the size of a vector can be achieved by
     * bounding r within [r_lower, r_upper]. Bounding theta within [-pi, -pi] or within [pi, pi]
     * is equivalent.
     * 
     * @author Justin Babilino
     */
    class PlanarBound {
    public:
        /**
         * @brief Enumerates the supported coordinate system types for planar bounds.
         * This includes rectangular (Cartesian) and polar coordinates.
         */
        enum class CoordinateType {
            /**
             * @brief used to indicate rectangular coordinates in planar bounds
             */
            kRectangular,
            /**
             * @brief used to indicate polar coordinates in planar bounds
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
         * @brief Construct a Planar Bound with the boundaries of both abstract coordinates
         * and the type of coordinates
         * 
         * @param a0 the bounds on the first abstract coordinate
         * @param a1 the bounds on the second abstract coordinate
         * @param coordinateType the coordinate type
         */
        PlanarBound(const ScalarBound& a0, const ScalarBound& a1, CoordinateType coordinateType);

        /**
         * @brief Check if this planar bound is valid. A planar bound is valid when the bounds
         * on a0 and a1 are valid, and the bounds of a1 are between -pi and pi, inclusive if
         * coordinateType is kPolar.
         * 
         * @return true if and only if this planar bound is valid
         */
        bool IsValid() const noexcept;
    };
}