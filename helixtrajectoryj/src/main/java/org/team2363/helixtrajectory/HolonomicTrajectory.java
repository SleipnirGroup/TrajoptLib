package org.team2363.helixtrajectory;

import java.util.List;

public final class HolonomicTrajectory extends Trajectory {

    public final List<HolonomicTrajectorySegment> holonomicSegments;

    private HolonomicTrajectory(List<HolonomicTrajectorySegment> holonomicSegments) {
        super(holonomicSegments);

        this.holonomicSegments = holonomicSegments;
    }

    public HolonomicTrajectory(HolonomicTrajectorySegment... holonomicSegments) throws NullPointerException {
        this(List.of(holonomicSegments));
    }
}