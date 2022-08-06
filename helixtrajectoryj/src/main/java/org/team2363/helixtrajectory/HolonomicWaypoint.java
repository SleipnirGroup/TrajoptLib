package org.team2363.helixtrajectory;

import java.util.List;

public final class HolonomicWaypoint extends Waypoint {

    public final double velocityX;
    public final double velocityY;
    public final double angularVelocity;

    public final boolean velocityXConstrained;
    public final boolean velocityYConstrained;
    public final boolean velocityMagnitudeConstrained;
    public final boolean angularVelocityConstrained;
    
    private HolonomicWaypoint(double x, double y, double heading,
            double velocityX, double velocityY, double angularVelocity,
            boolean xConstrained, boolean yConstrained, boolean headingConstrained,
            boolean velocityXConstrained, boolean velocityYConstrained,
            boolean velocityMagnitudeConstrained, boolean angularVelocityConstrained,
            List<? extends InitialGuessPoint> initialGuessPoints) throws NullPointerException {
        super(x, y, heading, xConstrained, yConstrained, headingConstrained, initialGuessPoints);

        this.velocityX = velocityX;
        this.velocityY = velocityY;
        this.angularVelocity = angularVelocity;

        this.velocityXConstrained = velocityXConstrained;
        this.velocityYConstrained = velocityYConstrained;
        this.velocityMagnitudeConstrained = velocityMagnitudeConstrained;
        this.angularVelocityConstrained = angularVelocityConstrained;
    }

    public HolonomicWaypoint(double x, double y, double heading,
            double velocityX, double velocityY, double angularVelocity,
            boolean xConstrained, boolean yConstrained, boolean headingConstrained,
            boolean velocityXConstrained, boolean velocityYConstrained,
            boolean velocityMagnitudeConstrained, boolean angularVelocityConstrained,
            InitialGuessPoint... initialGuessPoints) throws NullPointerException {
        this(x, y, heading, velocityX, velocityY, angularVelocity,
                xConstrained, yConstrained, headingConstrained,
                velocityXConstrained, velocityYConstrained,
                velocityMagnitudeConstrained, angularVelocityConstrained, List.of(initialGuessPoints));
    }
}