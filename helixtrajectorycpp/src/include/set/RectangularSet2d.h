#pragma once

#include <limits>
#include <vector>

#include "set/IntervalSet1d.h"

namespace helixtrajectory {

class RectangularSet2d {
public:
    IntervalSet1d xBound;
    IntervalSet1d yBound;

    RectangularSet2d(const IntervalSet1d& xBound, const IntervalSet1d& yBound);
    static RectangularSet2d PolarExactSet2d(double r, double theta);
    static RectangularSet2d R2();

    template<typename Expression>
    std::vector<decltype(Expression() == Expression())> GetConstraints(const Expression& x, const Expression& y) const {
        std::vector<decltype(Expression() == Expression())> constraints;
        std::vector<decltype(Expression() == Expression())> xConstraints = xBound.GetConstraints(x);
        std::vector<decltype(Expression() == Expression())> yConstraints = yBound.GetConstraints(y);
        constraints.reserve(xConstraints.size() + yConstraints.size());
        for (auto& xConstraint : xConstraints) {
            constraints.push_back(xConstraint);
        }
        for (auto& yConstraint : yConstraints) {
            constraints.push_back(yConstraint);
        }
        return constraints;
    }

    void CheckVector(double x, double y) const;

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