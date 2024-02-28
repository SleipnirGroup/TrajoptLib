// Copyright (c) TrajoptLib contributors

#include <stdexcept>

#include <emscripten/bind.h>
#include <trajopt/OptimalTrajectoryGenerator.h>
#include <trajopt/drivetrain/SwerveDrivetrain.h>
#include <trajopt/trajectory/HolonomicTrajectory.h>
#include <trajopt/trajectory/HolonomicTrajectorySample.h>

using namespace emscripten;

trajopt::HolonomicTrajectory opticall(int intervals) {
  trajopt::SwerveDrivetrain swerveDrivetrain{
      .mass = 45,
      .moi = 6,
      .modules = {{+0.6, +0.6, 0.04, 70, 2},
                  {+0.6, -0.6, 0.04, 70, 2},
                  {-0.6, +0.6, 0.04, 70, 2},
                  {-0.6, -0.6, 0.04, 70, 2}}};

  trajopt::Obstacle bumpers(
      0, {{+0.5, +0.5}, {-0.5, +0.5}, {-0.5, -0.5}, {+0.5, -0.5}});

  // One Meter Forward
  trajopt::SwervePathBuilder path;
  path.SetDrivetrain(swerveDrivetrain);
  path.PoseWpt(0, 0.0, 0.0, 0.0);
  path.PoseWpt(1, 5.0, 0.0, 0.0);
  path.WptZeroVelocity(0);
  path.WptZeroVelocity(1);
  path.ControlIntervalCounts({4});

  // SOLVE
  if (auto sol = trajopt::OptimalTrajectoryGenerator::Generate(path);
      sol.has_value()) {
    return trajopt::HolonomicTrajectory{sol.value()};
  } else {
    throw std::runtime_error(sol.error());
  }
}

EMSCRIPTEN_BINDINGS(my_module) {
  using namespace trajopt;

  function("opticall", &opticall);
  value_object<HolonomicTrajectorySample>("HolonomicTrajectorySample")
      .field("timestamp", &HolonomicTrajectorySample::timestamp)
      .field("x", &HolonomicTrajectorySample::x)
      .field("y", &HolonomicTrajectorySample::y)
      .field("heading", &HolonomicTrajectorySample::heading)
      .field("velocityX", &HolonomicTrajectorySample::velocityX)
      .field("velocityY", &HolonomicTrajectorySample::velocityY)
      .field("angularVelocity", &HolonomicTrajectorySample::angularVelocity);
  value_object<HolonomicTrajectory>("HolonomicTrajectory")
      .field("samples", &HolonomicTrajectory::samples);
  register_vector<HolonomicTrajectorySample>("HolonomicTrajectorySamples");
}
