package org.team2363.helixtrajectory;

public class Waypoint {

    public final double x, y, heading, vx, vy, omega;
    public final boolean xConstrained, yConstrained, headingConstrained,
            vxConstrained, vyConstrained, omegaConstrained;

    public Waypoint(double x, double y, double heading, double vx, double vy, double omega,
            boolean xConstrained, boolean yConstrained, boolean headingConstrained,
            boolean vxConstrained, boolean vyConstrained, boolean omegaConstrained) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
        
        this.xConstrained = xConstrained;
        this.yConstrained = yConstrained;
        this.headingConstrained = headingConstrained;
        this.vxConstrained = vxConstrained;
        this.vyConstrained = vyConstrained;
        this.omegaConstrained = omegaConstrained;
    }

    public static Waypoint softWaypoint(double x, double y) {
        return new Waypoint(x, y, 0.0, 0.0, 0.0, 0.0, true, true, false, false, false, false);
    }
    public static Waypoint hardWaypoint(double x, double y, double heading) {
        return new Waypoint(x, y, heading, 0.0, 0.0, 0.0, true, true, true, false, false, false);
    }

}