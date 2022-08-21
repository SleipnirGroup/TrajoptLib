package org.team2363.helixtrajectory;

import java.util.List;

public abstract class TrajectorySegment {

    public final double intervalDuration;

    public final List<? extends TrajectorySample> samples;

    protected TrajectorySegment(double intervalDuration, List<? extends TrajectorySample> samples) {
        this.intervalDuration = intervalDuration;

        this.samples = samples;
    }
}