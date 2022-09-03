package org.team2363.helixtrajectory;

public final class InitialGuessPoint {

    public final double x;
    public final double y;
    public final double heading;

    public InitialGuessPoint(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    @Override
    public String toString() {
        return "{\"x\": " + x
            + ", \"y\": " + y
            + ", \"heading\": " + heading
            + "}";
    }
}