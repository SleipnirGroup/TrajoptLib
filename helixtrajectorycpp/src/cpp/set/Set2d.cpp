// Copyright (c) TrajoptLib contributors

#include "set/Set2d.h"

#include <optional>
#include <variant>

#include "solution/SolutionChecking.h"

namespace helixtrajectory {

std::optional<SolutionError> Set2d::CheckVector(
    double xComp, double yComp, const SolutionTolerances& tolerances) const {
  if (IsRectangular()) {
    return GetRectangular().CheckVector(xComp, yComp, tolerances);
  } else if (IsLinear()) {
    return GetLinear().CheckVector(xComp, yComp, tolerances);
  } else if (IsElliptical()) {
    return GetElliptical().CheckVector(xComp, yComp, tolerances);
  } else /*if (IsCone())*/ {
    return GetCone().CheckVector(xComp, yComp, tolerances);
  }
}

bool Set2d::IsRectangular() const {
  return std::holds_alternative<RectangularSet2d>(set2d);
}
bool Set2d::IsLinear() const {
  return std::holds_alternative<LinearSet2d>(set2d);
}
bool Set2d::IsElliptical() const {
  return std::holds_alternative<EllipticalSet2d>(set2d);
}
bool Set2d::IsCone() const {
  return std::holds_alternative<ConeSet2d>(set2d);
}

const RectangularSet2d& Set2d::GetRectangular() const {
  return std::get<RectangularSet2d>(set2d);
}
RectangularSet2d& Set2d::GetRectangular() {
  return std::get<RectangularSet2d>(set2d);
}
const LinearSet2d& Set2d::GetLinear() const {
  return std::get<LinearSet2d>(set2d);
}
LinearSet2d& Set2d::GetLinear() {
  return std::get<LinearSet2d>(set2d);
}
const EllipticalSet2d& Set2d::GetElliptical() const {
  return std::get<EllipticalSet2d>(set2d);
}
EllipticalSet2d& Set2d::GetElliptical() {
  return std::get<EllipticalSet2d>(set2d);
}
const ConeSet2d& Set2d::GetCone() const {
  return std::get<ConeSet2d>(set2d);
}
ConeSet2d& Set2d::GetCone() {
  return std::get<ConeSet2d>(set2d);
}

Set2d::Set2d(const RectangularSet2d& rectangularSet2d)
    : set2d(rectangularSet2d) {}
Set2d::Set2d(const LinearSet2d& linearSet2d) : set2d(linearSet2d) {}
Set2d::Set2d(const EllipticalSet2d& ellipticalSet2d) : set2d(ellipticalSet2d) {}
Set2d::Set2d(const ConeSet2d& coneSet2d) : set2d(coneSet2d) {}
}  // namespace helixtrajectory
