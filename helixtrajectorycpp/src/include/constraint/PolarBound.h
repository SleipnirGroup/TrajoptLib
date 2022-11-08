#pragma once

#include <cmath>
#include <limits>

#include "constraint/ScalarBound.h"

namespace helixtrajectory {

    class PolarBound {
    public:
        ScalarBound r;
        ScalarBound theta;

        /**
         * @brief Construct a Planar Bound with the boundaries of abstract coordinates
         * a0 and a1 and the type of coordinates. By default, an infinite bound is created.
         * 
         * @param a0 the bounds on the first abstract coordinate
         * @param a1 the bounds on the second abstract coordinate
         * @param coordinateType the coordinate type
         */
        constexpr PolarBound(const ScalarBound& r,
                const ScalarBound& theta);

        constexpr PolarBound CircleBound(double r) noexcept;
        constexpr PolarBound LineBound(double theta) noexcept;

        /**
         * @brief Check if this planar bound is circularly symmetric. A planar bound is
         * circularly symmetric if applying a rotation transformation about the origin to
         * the bound would not change the bound. Therefore, circularly symemetry occurs
         * when the bound is polar and a1 has a range of 2*pi or when this bound is zero.
         * 
         * @return IsZero() || (IsPolar() && a1.Range() >= 2*pi) || IsInfinite()
         */
        bool IsCircularlySymmetric() const noexcept;

        /**
         * @brief Check if this planar bound only contains one point. This
         * occurs when a0 and a1 are exact.
         * 
         * @return IsZero() || (r.IsExact() && theta.IsExact())
         */
        bool IsExact() const noexcept;

        /**
         * @brief Check if this planar bound only contains the origin. This
         * occurs for rectangular bounds when a0 and a1 are both zero and for
         * polar bounds when a0 is zero.
         * 
         * @return r == 0
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