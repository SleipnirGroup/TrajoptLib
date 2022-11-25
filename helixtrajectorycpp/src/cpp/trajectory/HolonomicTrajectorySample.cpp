#include "trajectory/HolonomicTrajectorySample.h"

#include <iostream>

#include <fmt/format.h>

#include "IncompatibleTrajectoryException.h"
#include "trajectory/HolonomicState.h"
#include "TestUtil.h"

namespace helixtrajectory {

HolonomicTrajectorySample::HolonomicTrajectorySample(double intervalDuration, const HolonomicState& state)
        : intervalDuration(intervalDuration), state(state) {
}

void HolonomicTrajectorySample::CheckKinematics(const HolonomicState& previousState) const {
    if (auto currentStateX = previousState.x + state.velocityX * intervalDuration; !WithinPrecision(state.x, currentStateX, 1e-3))
        throw IncompatibleTrajectoryException(fmt::format("previous x of {} + velocity x of {} * interval duration of {} = x of {}, but the current sample has an x of {}.",
                previousState.x, state.velocityX, intervalDuration, currentStateX, state.x));
    if (auto currentStateY = previousState.y + state.velocityY * intervalDuration; !WithinPrecision(state.y, currentStateY, 1e-3))
        throw IncompatibleTrajectoryException(fmt::format("previous y of {} + velocity y of {} * interval duration of {} = y of {}, but the current sample has an y of {}.",
                previousState.y, state.velocityY, intervalDuration, currentStateY, state.y));
    if (auto currentStateHeading = previousState.heading + state.angularVelocity * intervalDuration; !WithinPrecision(state.heading, currentStateHeading, 1e-3))
        throw IncompatibleTrajectoryException(fmt::format("previous heading of {} + angular velocity of {} * interval duration of {} = heading of {}, but the current sample has a heading of {}.",
                previousState.heading, state.angularVelocity, intervalDuration, currentStateHeading, state.heading));

    if (auto currentStateVelocityX = previousState.velocityX + state.accelerationX * intervalDuration; !WithinPrecision(state.velocityX, currentStateVelocityX, 1e-3))
        throw IncompatibleTrajectoryException(fmt::format("previous velocity x of {} + acceleration x of {} * interval duration of {} = velocity x of {}, but the current sample has a velocity x of {}.",
                previousState.velocityX, state.accelerationX, intervalDuration, currentStateVelocityX, state.velocityX));
    if (auto currentStateVelocityY = previousState.velocityY + state.accelerationY * intervalDuration; !WithinPrecision(state.velocityY, currentStateVelocityY, 1e-3))
        throw IncompatibleTrajectoryException(fmt::format("previous velocity y of {} + acceleration y of {} * interval duration of {} = velocity y of {}, but the current sample has a velocity y of {}.",
                previousState.velocityY, state.accelerationY, intervalDuration, currentStateVelocityY, state.velocityY));
    if (auto currentStateAngularVelocity = previousState.angularVelocity + state.angularAcceleration * intervalDuration; !WithinPrecision(state.angularVelocity, currentStateAngularVelocity, 1e-3))
        throw IncompatibleTrajectoryException(fmt::format("previous angular velocity of {} + angular acceleration of {} * interval duration of {} = angular velocity of {}, but the current sample has an angular velocity of {}.",
                previousState.angularVelocity, state.angularAcceleration, intervalDuration, currentStateAngularVelocity, state.angularVelocity));
}

std::ostream& operator<<(std::ostream& stream, const HolonomicTrajectorySample& sample) {
    return stream << "{\"interval_duration\": " << sample.intervalDuration
            << ", \"state\": " << sample.state
            << "}";
}
}