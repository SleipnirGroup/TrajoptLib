#include "TrajectorySample.h"

namespace helixtrajectory {

    TrajectorySample::TrajectorySample(double x, double y, double heading)
            : x(x), y(y), heading(heading) {
    }

    TrajectorySample::~TrajectorySample() {
    }
}