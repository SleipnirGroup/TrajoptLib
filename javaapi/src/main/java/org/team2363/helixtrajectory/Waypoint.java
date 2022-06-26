package org.team2363.helixtrajectory;

public class Waypoint {

    static final int ARRAY_LENGTH = 3;

    public final double x, y, heading;

    public Waypoint(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}
