#pragma once

#include <limits>

#include "set/IntervalSet1d.h"

namespace helixtrajectory {

    class RectangularSet2d {
    public:
        IntervalSet1d xBound;
        IntervalSet1d yBound;

        RectangularSet2d(const IntervalSet1d& xBound, const IntervalSet1d& yBound);

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