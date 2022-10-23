#include "constraint/PlanarBound.h"

#include "constraint/ScalarBound.h"

#include <cmath>

namespace helixtrajectory {

    PlanarBound::PlanarBound(const ScalarBound& a0, const ScalarBound& a1, CoordinateType coordinateType)
            : a0(a0), a1(a1), coordinateType(coordinateType) {
    }

    bool PlanarBound::IsRectangular() const noexcept {
        return coordinateType == CoordinateType::kRectangular;
    }

    bool PlanarBound::IsPolar() const noexcept {
        return coordinateType == CoordinateType::kPolar;
    }

    bool PlanarBound::IsCircularlySymmetric() const noexcept {
        return IsInfinite() || IsZero() || (IsPolar() && a1.Range() >= 2 * M_PI);
    }

    bool PlanarBound::IsExact() const noexcept {
        return IsZero() || (a0.IsExact() && a1.IsExact());
    }

    bool PlanarBound::IsZero() const noexcept {
        return a0.IsZero() && (!IsRectangular() || a1.IsZero());
    }

    bool PlanarBound::IsInfinite() const noexcept {
        return a0.IsInfinite() && ((IsRectangular() && a1.IsInfinite()) || (IsPolar() && a1.Range() >= 2 * M_PI));
    }

    bool PlanarBound::IsValid() const noexcept {
        return a0.IsValid() && a1.IsValid() && (IsRectangular() || (IsPolar() && a1.lower >= -M_PI && a1.upper <= M_PI));
    }
}