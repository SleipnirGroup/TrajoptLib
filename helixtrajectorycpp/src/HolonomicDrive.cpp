#include "HolonomicDrive.h"

namespace helixtrajectory {

    HolonomicDrive::HolonomicDrive(double mass, double moi, const Obstacle& bumpers)
        : Drive(mass, moi, bumpers) {
    }
}