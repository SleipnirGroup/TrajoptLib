#include "HolonomicTrajectorySample.h"

#include <iostream>

#include "TrajectorySample.h"

namespace helixtrajectory {

    HolonomicTrajectorySample::HolonomicTrajectorySample(double intervalDuration,
            double x, double y, double heading,
            double velocityX, double velocityY, double angularVelocity) 
            : TrajectorySample(intervalDuration, x, y, heading),
            velocityX(velocityX), velocityY(velocityY), angularVelocity(angularVelocity) {
    }

    HolonomicTrajectorySample::~HolonomicTrajectorySample() {
    }

    std::ostream& operator<<(std::ostream& stream, const HolonomicTrajectorySample& sample) {
        return stream << "{\"x\": " << sample.x
                << ", \"y\": " << sample.y
                << ", \"heading\": " << sample.heading
                << ", \"velocity_x\": " << sample.velocityX
                << ", \"velocity_y\": " << sample.velocityY
                << ", \"angular_velocity\": " << sample.angularVelocity
                << "}";
    }
}