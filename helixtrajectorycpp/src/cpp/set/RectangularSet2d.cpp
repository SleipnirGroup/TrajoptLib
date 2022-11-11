#include "set/RectangularSet2d.h"

#include <cmath>

#include "set/IntervalSet1d.h"

namespace helixtrajectory {

    RectangularSet2d::RectangularSet2d(const IntervalSet1d& xBound, const IntervalSet1d& yBound)
            : xBound(xBound), yBound(yBound) {
    }

    RectangularSet2d RectangularSet2d::PolarExactSet2d(double r, double theta) {
        return RectangularSet2d(r*cos(theta), r*sin(theta));
    }

    RectangularSet2d RectangularSet2d::R2() {
        return RectangularSet2d(IntervalSet1d::R1(), IntervalSet1d::R1());
    }

    bool RectangularSet2d::IsValid() const noexcept {
        return xBound.IsValid() && yBound.IsValid();
    }
}