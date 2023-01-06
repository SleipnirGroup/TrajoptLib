// Copyright (c) TrajoptLib contributors

package org.team2363.helixtrajectory;

public final class ObstaclePoint {
    public final double x;
    public final double y;

    public ObstaclePoint(double x, double y) {
        this.x = x;
        this.y = y;
    }

    @Override
    public String toString() {
        return "{\"x\": " + x
            + ", \"y\": " + y
            + "}";
    }
}
