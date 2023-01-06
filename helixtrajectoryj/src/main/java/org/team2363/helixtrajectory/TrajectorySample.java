// Copyright (c) TrajoptLib contributors

package org.team2363.helixtrajectory;

public abstract class TrajectorySample {
    public final double intervalDuration;

    public final double x;
    public final double y;
    public final double heading;

    protected TrajectorySample(double intervalDuration, double x, double y, double heading) {
        this.intervalDuration = intervalDuration;

        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}
