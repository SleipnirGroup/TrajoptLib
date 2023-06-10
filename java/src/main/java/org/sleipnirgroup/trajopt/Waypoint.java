// Copyright (c) TrajoptLib contributors

package org.sleipnirgroup.trajopt;

import static org.sleipnirgroup.util.ObjectChecker.requireNonNegative;
import static org.sleipnirgroup.util.ObjectChecker.requireNonNullAndWrapUnmodifiable;

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
            List<? extends InitialGuessPoint> initialGuessPoints,
            List<? extends Obstacle> obstacles) throws NullPointerException, IllegalArgumentException {
        this.x = x;
        this.y = y;
        this.heading = heading;

        this.xConstrained = xConstrained;
        this.yConstrained = yConstrained;
        this.headingConstrained = headingConstrained;

        this.controlIntervalCount = requireNonNegative(controlIntervalCount, "Control interval count cannot be negative");

        this.initialGuessPoints = requireNonNullAndWrapUnmodifiable(initialGuessPoints,
                "List of initial guess points cannot be null", "Initial guess point cannot be null");
    }

    public boolean isInitialWaypoint() {
        return controlIntervalCount == 0;
    }

    public boolean isValid() {
        return (isInitialWaypoint() && initialGuessPoints.isEmpty())
                || (!isInitialWaypoint() && initialGuessPoints.size() < controlIntervalCount);
    }

    public boolean isPositionStateKnown() {
        return xConstrained && yConstrained && headingConstrained;
    }

    public abstract boolean isVelocityStateKnown();

    public boolean isStateKnown() {
        return isPositionStateKnown() && isVelocityStateKnown();
    }

    public boolean isSplitWaypoint() {
        return isInitialWaypoint() && isStateKnown();
    }
}
