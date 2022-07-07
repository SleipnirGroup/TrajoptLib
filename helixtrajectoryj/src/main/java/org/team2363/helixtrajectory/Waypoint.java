package org.team2363.helixtrajectory;

public class Waypoint {

    public final double x, y, heading, vx, vy, omega;
    public final boolean xConstrained, yConstrained, headingConstrained,
            vxConstrained, vyConstrained, vMagnitudeConstrained, omegaConstrained;
    public final InitialGuessPoint[] initialGuessPoints;

    public Waypoint(double x, double y, double heading, double vx, double vy, double omega,
            boolean xConstrained, boolean yConstrained, boolean headingConstrained,
            boolean vxConstrained, boolean vyConstrained, boolean vMagnitudeConstrained, boolean omegaConstrained,
            InitialGuessPoint... initialGuessPoints) {
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
        this.vMagnitudeConstrained = vMagnitudeConstrained;
        this.omegaConstrained = omegaConstrained;

        this.initialGuessPoints = initialGuessPoints;
    }

    public int initialGuessPointsLength() {
        return initialGuessPoints.length;
    }

    public InitialGuessPoint getInitialGuessPoint(int index) throws IndexOutOfBoundsException {
        if (index >= 0 && index < initialGuessPointsLength()) {
            return initialGuessPoints[index];
        } else {
            throw new IndexOutOfBoundsException();
        }
    }

    public static Waypoint softWaypoint(double x, double y) {
        return new Waypoint(x, y, 0.0, 0.0, 0.0, 0.0, true, true, false, false, false, false, false);
    }
    public static Waypoint hardWaypoint(double x, double y, double heading) {
        return new Waypoint(x, y, heading, 0.0, 0.0, 0.0, true, true, true, false, false, false, false);
    }

}