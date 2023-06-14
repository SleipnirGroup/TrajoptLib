// Copyright (c) TrajoptLib contributors

package org.sleipnirgroup.trajopt;

public abstract class TrajectorySample {
    public final double timestamp;

    public final double x;
    public final double y;
    public final double heading;

    protected TrajectorySample(double timestamp, double x, double y, double heading) {
        this.timestamp = timestamp;

        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}
