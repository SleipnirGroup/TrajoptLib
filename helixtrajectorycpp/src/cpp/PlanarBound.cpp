#include "PlanarBound.h"

#include <cmath>

namespace helixtrajectory {

    PlanarBound::PlanarBound(const ScalarBound& a0, const ScalarBound& a1, CoordinateType coordinateType)
            : a0(a0), a1(a1), coordinateType(coordinateType) {
    }

    bool PlanarBound::IsValid() const noexcept {
        return a0.IsValid() && a1.IsValid() && (coordinateType == CoordinateType::kPolar && a1.lower >= -M_PI && a1.upper <= M_PI);
    }
}