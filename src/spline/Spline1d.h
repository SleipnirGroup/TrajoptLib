// Copyright (c) TrajoptLib contributors

#pragma once

namespace trajopt {

class TRAJOPT_DLLEXPORT Spline1d {
 public:
  virtual ~Spline1d() {}

  virtual double getPosition(double t) const = 0;

  // ds/dt
  virtual double getVelocity(double t) const = 0;

  // ds^2/dt
  virtual double getAcceleration(double t) const = 0;

  // ds^3/dt
  virtual double getJerk(double t) const = 0;
};
}  // namespace trajopt
