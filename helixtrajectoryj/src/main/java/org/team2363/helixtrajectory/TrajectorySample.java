package org.team2363.helixtrajectory;

public abstract class TrajectorySample {

    public final double x;
    public final double y;
    public final double heading;

    protected TrajectorySample(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}