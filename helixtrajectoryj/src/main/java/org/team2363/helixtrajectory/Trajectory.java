package org.team2363.helixtrajectory;

import java.util.List;

public abstract class Trajectory {
    
    public final List<? extends TrajectorySample> samples;

    protected Trajectory(List<? extends TrajectorySample> samples) {
        this.samples = samples;
    }
}