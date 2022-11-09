#pragma once

namespace helixtrajectory {

class EllipticalSet2d {
public:
    double xRadius;
    double yRadius;
    bool invert;

    EllipticalSet2d(double xRadius, double yRadius, bool invert = false);
    static EllipticalSet2d CircularSet2d(double radius, bool invert = false);

    bool IsCircular() const noexcept;
    bool IsR2() const noexcept;
    bool IsValid() const noexcept;
};
}