#include "set/LinearSet2d.h"

namespace helixtrajectory {

LinearSet2d::LinearSet2d(double theta, const IntervalSet1d& rBound)
        : theta(theta), rBound(rBound) {
}
}