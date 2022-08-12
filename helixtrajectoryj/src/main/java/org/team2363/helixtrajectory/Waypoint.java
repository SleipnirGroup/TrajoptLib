package org.team2363.helixtrajectory;

import java.util.Collections;
import java.util.List;

public abstract class Waypoint {

    public final double x;
    public final double y;
    public final double heading;
    public final boolean xConstrained;
    public final boolean yConstrained;
    public final boolean headingConstrained;
    public final int controlIntervalCount;

    public final List<? extends InitialGuessPoint> initialGuessPoints;

    protected Waypoint(double x, double y, double heading,
            boolean xConstrained, boolean yConstrained, boolean headingConstrained,
            int controlIntervalCount,
            List<? extends InitialGuessPoint> initialGuessPoints) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        
        this.xConstrained = xConstrained;
        this.yConstrained = yConstrained;
        this.headingConstrained = headingConstrained;

        this.controlIntervalCount = controlIntervalCount;

        this.initialGuessPoints = Collections.unmodifiableList(initialGuessPoints);
    }
}