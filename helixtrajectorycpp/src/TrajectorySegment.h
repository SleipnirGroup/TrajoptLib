#pragma once

namespace helixtrajectory {

    class TrajectorySegment {
    public:
        double dt;

        TrajectorySegment(double dt);

        virtual ~TrajectorySegment();
    };
}