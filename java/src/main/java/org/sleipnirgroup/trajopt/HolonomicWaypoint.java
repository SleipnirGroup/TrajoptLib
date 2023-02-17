// Copyright (c) TrajoptLib contributors

package org.sleipnirgroup.trajopt;

import java.util.List;

public final class HolonomicWaypoint extends Waypoint {
    public final double velocityX;
    public final double velocityY;
    public final double angularVelocity;

    public final boolean velocityXConstrained;
    public final boolean velocityYConstrained;
    public final boolean velocityMagnitudeConstrained;
    public final boolean angularVelocityConstrained;

    public HolonomicWaypoint(double x, double y, double heading,
            double velocityX, double velocityY, double angularVelocity,
            boolean xConstrained, boolean yConstrained, boolean headingConstrained,
            boolean velocityXConstrained, boolean velocityYConstrained,
            boolean velocityMagnitudeConstrained, boolean angularVelocityConstrained,
            int controlIntervalCount,
            List<? extends InitialGuessPoint> initialGuessPoints,
            List<? extends Obstacle> obstacles) throws NullPointerException, IllegalArgumentException {
        super(x, y, heading, xConstrained, yConstrained, headingConstrained,
                controlIntervalCount, initialGuessPoints, obstacles);

        this.velocityX = velocityX;
        this.velocityY = velocityY;
        this.angularVelocity = angularVelocity;

        this.velocityXConstrained = velocityXConstrained;
        this.velocityYConstrained = velocityYConstrained;
        this.velocityMagnitudeConstrained = velocityMagnitudeConstrained;
        this.angularVelocityConstrained = angularVelocityConstrained;
    }

    @Override
    public boolean isVelocityStateKnown() {
        return (velocityMagnitudeConstrained && velocityXConstrained && velocityYConstrained)
                || ((velocityX == 0.0 && velocityY == 0.0)
                && ((!velocityMagnitudeConstrained && velocityXConstrained && velocityYConstrained)
                || (velocityMagnitudeConstrained && !velocityXConstrained && !velocityYConstrained)));
    }

    @Override
    public String toString() {
        return "{\"x\": " + x
            + ", \"y\": " + y
            + ", \"heading\": " + heading
            + ", \"velociy_x\": " + velocityX
            + ", \"velocity_y\": " + velocityY
            + ", \"angular_velocity\": " + angularVelocity
            + ", \"x_constrained\": " + xConstrained
            + ", \"y_constrained\": " + yConstrained
            + ", \"heading_constrained\": " + headingConstrained
            + ", \"velocity_x_constrained\": " + velocityXConstrained
            + ", \"velocity_y_constrained\": " + velocityYConstrained
            + ", \"velocity_magnitude_constrained\": " + velocityMagnitudeConstrained
            + ", \"angular_velocity_constrained\": " + angularVelocityConstrained
            + ", \"control_interval_count\": " + controlIntervalCount
            + ", \"initial_guess_points\": " + initialGuessPoints
            + ", \"obstacles\": " + obstacles
            + "}";
    }
}
