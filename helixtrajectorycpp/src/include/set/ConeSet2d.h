#pragma once

#include "set/IntervalSet1d.h"

namespace helixtrajectory {

class ConeSet2d {
public:
    IntervalSet1d thetaBound;

    ConeSet2d(const IntervalSet1d& thetaBound);

    bool IsValid() const noexcept;
};
}