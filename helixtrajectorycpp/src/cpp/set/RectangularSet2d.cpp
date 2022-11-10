#include "set/RectangularSet2d.h"

#include "set/IntervalSet1d.h"

namespace helixtrajectory {

    RectangularSet2d::RectangularSet2d(const IntervalSet1d& xBound, const IntervalSet1d& yBound)
            : xBound(xBound), yBound(yBound) {
    }

    bool RectangularSet2d::IsValid() const noexcept {
        return xBound.IsValid() && yBound.IsValid();
    }
}