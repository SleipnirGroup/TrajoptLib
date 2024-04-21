// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include "frc/EigenCore.h"
#include "frc/MathUtil.h"
#include "frc/spline/CubicHermiteSpline.h"
#include "spline/CubicHermiteSpline1d.h"

namespace trajopt {
/**
 * Represents a cubic pose spline, which is a specific implementation of a cubic
 * hermite spline.
 */
class TRAJOPT_DLLEXPORT CubicHermitePoseSplineHolonomic
    : public frc::CubicHermiteSpline {
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

  CubicHermitePoseSplineHolonomic(frc::CubicHermiteSpline spline,
                                  frc::Rotation2d r0, frc::Rotation2d r1)
      : frc::CubicHermiteSpline(spline.GetInitialControlVector().x,
                                spline.GetFinalControlVector().x,
                                spline.GetInitialControlVector().y,
                                spline.GetFinalControlVector().y),
        r0(r0),
        theta(0.0, (-r0).RotateBy(r1).Radians().value(), 0, 0) {}

  ~CubicHermitePoseSplineHolonomic() noexcept {
  }  // there is an error without noexcept

  frc::Rotation2d getHeading(double t) const {
    return r0.RotateBy(frc::Rotation2d(units::radian_t(theta.getPosition(t))));
  }

  double getDHeading(double t) const { return theta.getVelocity(t); }

  /**
   * Gets the pose and curvature at some point t on the spline.
   *
   * @param t The point t
   * @return The pose and curvature at that point.
   */
  PoseWithCurvature GetPoint(double t) const {
    const auto base = frc::CubicHermiteSpline::GetPoint(t);
    return {{base.first.Translation(), getHeading(t)}, base.second};
  }

 private:
  const CubicHermiteSpline1d theta;
  const frc::Rotation2d r0;
};
}  // namespace trajopt
