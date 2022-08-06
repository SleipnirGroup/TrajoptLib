#include "HolonomicDrive.h"

#include "Drive.h"
#include "Obstacle.h"

namespace helixtrajectory {

    HolonomicDrive::HolonomicDrive(double mass, double moi, const Obstacle& bumpers)
        : Drive(mass, moi, bumpers) {
    }

    HolonomicDrive::~HolonomicDrive() {
    }
}