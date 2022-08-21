#pragma once

namespace helixtrajectory {

    class TrajectorySegment {
    public:
        double intervalDuration;

        TrajectorySegment(double dt);

        virtual ~TrajectorySegment();
    };
}