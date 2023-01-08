// Copyright (c) TrajoptLib contributors

#include "drivetrain/SwerveDrivetrain.h"

#include <cmath>
#include <cstddef>
#include <iostream>
#include <string>
#include <vector>

#include <fmt/format.h>

#include "IncompatibleTrajectoryException.h"
#include "drivetrain/HolonomicDrivetrain.h"
#include "obstacle/Obstacle.h"

namespace trajopt {

SwerveDrivetrain::SwerveDrivetrain(double mass, double momentOfInertia,
                                   std::vector<SwerveModule> modules)
    : HolonomicDrivetrain(mass, momentOfInertia), modules(std::move(modules)) {}

// void SwerveDrivetrain::CheckState(const HolonomicState& state) const {
//     for (size_t moduleIndex = 0; moduleIndex < modules.size(); moduleIndex++)
//     {
//         auto& _module = modules[moduleIndex];

//         std::array<double, 2> rotatedVelocity =
//         ApplyRotation({state.velocityX, state.velocityY}, -state.heading);
//         auto moduleVelocity = _module.CalculateVelocity(rotatedVelocity[0],
//         rotatedVelocity[1], state.angularVelocity); double
//         velocityMagnitudeSquared = moduleVelocity[0] * moduleVelocity[0] +
//         moduleVelocity[1] * moduleVelocity[1]; double maxVelocityMagnitude =
//         _module.wheelMaxAngularVelocity * _module.wheelRadius; double
//         maxVelocityMagnitudeSquared = maxVelocityMagnitude *
//         maxVelocityMagnitude;

//         if (!(velocityMagnitudeSquared <= maxVelocityMagnitudeSquared)) {
//             throw IncompatibleTrajectoryException(fmt::format(
//                     "wheel of module {} must have an angular velocity less
//                     than or equal to {}, but the trajectory requires {}.",
//                     moduleIndex, maxVelocityMagnitude,
//                     std::sqrt(velocityMagnitudeSquared)));
//         }
//     }
// }

// void SwerveDrivetrain::CheckTrajectory(const HolonomicTrajectory& trajectory)
// const {
//     for (size_t sampleIndex = 0; sampleIndex < trajectory.samples.size() + 1;
//     sampleIndex++) {
//         const HolonomicState* state;
//         if (sampleIndex == 0) {
//             state = &trajectory.initialState;
//         } else {
//             state = &trajectory.samples[sampleIndex - 1].state;
//         }
//         try {
//             CheckState(*state);
//         } catch (const IncompatibleTrajectoryException& exception) {
//             throw IncompatibleTrajectoryException(fmt::format("At trajectory
//             sample index {}, {}", sampleIndex, exception.what()));
//         }
//     }
// }

std::ostream& operator<<(std::ostream& stream,
                         const SwerveDrivetrain& swerveDrivetrain) {
  return stream << "{\"mass\": " << swerveDrivetrain.mass
                << ", \"moment_of_inertia\": "
                << swerveDrivetrain.momentOfInertia
                //    << ", \"modules\": " << swerveDrivetrain.modules
                << "}";
}
}  // namespace trajopt
