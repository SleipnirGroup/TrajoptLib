#pragma once

#include <array>

#include <casadi/casadi.hpp>

#include "set/IntervalSet1d.h"

namespace helixtrajectory {

class ConeSet2d {
public:
    IntervalSet1d thetaBound;

    ConeSet2d(const IntervalSet1d& thetaBound);

    template<typename Expression>
    std::array<decltype(Expression() == Expression()), 2> GetConstraints(const Expression& x, const Expression& y) const {
        return {x * sin(thetaBound.upper) >= y * cos(thetaBound.upper),
                x * sin(thetaBound.lower) <= y * cos(thetaBound.lower)};
    }

    void CheckVector(double x, double y) const;

    bool IsValid() const noexcept;
};
}