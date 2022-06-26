package org.team2363.helixtrajectory;

public class TrajectorySample {
    public final double ts, x, y, heading, vx, vy, omega;

    public TrajectorySample(double ts, double x, double y,
            double heading, double vx, double vy, double omega) {
        this.ts = ts;
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }
}
