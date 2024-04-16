// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include "frc/spline/CubicHermiteSpline.h" 
#include "frc/MathUtil.h"

#include "spline/CubicHermiteSpline1d.h"
#include <EigenCore.h>

namespace trajopt {
/**
 * Represents a cubic pose spline, which is a specific implementation of a cubic hermite spline.
 */
class CubicHermitePoseSplineHolonomic : public frc::CubicHermiteSpline {
public:
    CubicHermitePoseSplineHolonomic(wpi::array<double, 2> xInitialControlVector,
                    wpi::array<double, 2> xFinalControlVector,
                    wpi::array<double, 2> yInitialControlVector,
                    wpi::array<double, 2> yFinalControlVector,
                    frc::Rotation2d r0, frc::Rotation2d r1)
        : frc::CubicHermiteSpline(xInitialControlVector, xFinalControlVector,
                             yInitialControlVector, yFinalControlVector), 
                             r0(r0), 
                             theta(0.0, (-r0).RotateBy(r1).Radians().value(), 0, 0) {}

    ~CubicHermitePoseSplineHolonomic() noexcept {} // there is an error without noexcept

    frc::Rotation2d getHeading(double t) const {
        return r0.RotateBy(frc::Rotation2d(units::radian_t(theta.getPosition(t))));
    }

    double getDHeading(double t) const {
        return theta.getVelocity(t);
    }

  /**
   * Gets the pose and curvature at some point t on the spline.
   *
   * @param t The point t
   * @return The pose and curvature at that point.
   */
  PoseWithCurvature GetPoint(double t) const {
    const int Degree = 3;

    frc::Vectord<Degree + 1> polynomialBases;

    // Populate the polynomial bases
    for (int i = 0; i <= Degree; i++) {
      polynomialBases(i) = std::pow(t, Degree - i);
    }

    // This simply multiplies by the coefficients. We need to divide out t some
    // n number of times where n is the derivative we want to take.
    frc::Vectord<6> combined = Coefficients() * polynomialBases;

    double dx, dy, ddx, ddy;

    // If t = 0, all other terms in the equation cancel out to zero. We can use
    // the last x^0 term in the equation.
    if (t == 0.0) {
      dx = Coefficients()(2, Degree - 1);
      dy = Coefficients()(3, Degree - 1);
      ddx = Coefficients()(4, Degree - 2);
      ddy = Coefficients()(5, Degree - 2);
    } else {
      // Divide out t for first derivative.
      dx = combined(2) / t;
      dy = combined(3) / t;

      // Divide out t for second derivative.
      ddx = combined(4) / t / t;
      ddy = combined(5) / t / t;
    }

    // Find the curvature.
    const auto curvature =
        (dx * ddy - ddx * dy) / ((dx * dx + dy * dy) * std::hypot(dx, dy));

    return {
        {FromVector(combined.template block<2, 1>(0, 0)), getHeading(t)},
        units::curvature_t{curvature}};
  }

 private:
    const CubicHermiteSpline1d theta;
    const frc::Rotation2d r0;
};
}  // namespace frc