#include "ConeSet2d.h"

#include <cmath>

namespace helixtrajectory {

ConeSet2d::ConeSet2d(const IntervalSet1d& thetaBound)
        : thetaBound(thetaBound);

ConeSet2d::IsValid() const noexcept {
    return thetaBound.Range() > 0.0 && thetaBound.Range() <= M_PI;
}
}