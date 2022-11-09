#include "set/RectangularSet2d.h"

#include "set/IntervalSet1d.h"

namespace helixtrajectory {

    RectangularSet2d::RectangularSet2d(const ScalarBound& x, const ScalarBound& y)
            : x(x), y(y) {
    }

    RectangularSet2d::IsValid() const noexcept {
        return x.IsValid() && y.IsValid();
    }
}