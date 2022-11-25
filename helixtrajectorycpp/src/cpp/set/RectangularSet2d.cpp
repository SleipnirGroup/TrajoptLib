#include "set/RectangularSet2d.h"

#include <cmath>

#include <fmt/format.h>

#include "set/IntervalSet1d.h"
#include "IncompatibleTrajectoryException.h"

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

    void RectangularSet2d::CheckVector(double x, double y) const {
        try {
            xBound.CheckScalar(x);
        } catch (const IncompatibleTrajectoryException& exception) {
            throw IncompatibleTrajectoryException(fmt::format("x of {}", exception.what()));
        }
        try {
            yBound.CheckScalar(y);
        } catch (const IncompatibleTrajectoryException& exception) {
            throw IncompatibleTrajectoryException(fmt::format("y of {}", exception.what()));
        }
    }

    bool RectangularSet2d::IsValid() const noexcept {
        return xBound.IsValid() && yBound.IsValid();
    }
}