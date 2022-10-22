#include "PlanarBound.h"
#include "ScalarBound.h"

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
        return IsZero() || (IsPolar() && a1.Range() >= 2 * M_PI);
    }

    bool PlanarBound::IsOnXAxis() const noexcept {
        return (IsRectangular() && a1.IsZero()) || (IsPolar() && (a1 == -M_PI || a1 == 0.0 || a1 == M_PI));
    }
    
    bool PlanarBound::IsOnYAxis() const noexcept {
        return (IsRectangular() && a0.IsZero()) || (IsPolar() && (a1 == -M_PI_2 || a1 == M_PI_2));
    }

    bool PlanarBound::IsLinear() const noexcept {
        return !IsExact() && ((IsPolar() && a1.IsExact()) || (IsOnXAxis() || IsOnYAxis()));
    }

    bool PlanarBound::IsExact() const noexcept {
        return IsZero() || (a0.IsExact() && a1.IsExact());
    }

    bool PlanarBound::IsZero() const noexcept {
        return a0.IsZero() && (!IsRectangular() || a1.IsZero());
    }

    bool PlanarBound::IsInfinite() const noexcept {
        return a0.IsInfinite() && a1.IsInfinite();
    }

    bool PlanarBound::IsValid() const noexcept {
        return a0.IsValid() && a1.IsValid() && (IsRectangular() || (IsPolar() && a0.lower >= 0.0 && a1.lower >= -M_PI && a1.upper <= M_PI));
    }
}