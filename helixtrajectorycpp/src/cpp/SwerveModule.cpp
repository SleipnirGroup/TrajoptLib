#include "SwerveModule.h"

namespace helixtrajectory {

    std::ostream& operator<<(std::ostream& stream, const SwerveModule& module) {
        return stream << "{\"x\": " << module.x
                << ", \"y\": " << module.y
                << ", \"wheel_radius\": " << module.wheelRadius
                << ", \"wheel_max_angular_velocity\": " << module.wheelMaxAngularVelocity
                << ", \"wheel_max_torque\": " << module.wheelMaxTorque
                << "}";
    }
}