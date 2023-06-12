// Copyright (c) TrajoptLib contributors

package org.sleipnirgroup.trajopt;

public class HolonomicTrajectorySample extends TrajectorySample {
    public final double velocityX;
    public final double velocityY;
    public final double angularVelocity;

    public HolonomicTrajectorySample(double timestamp,
            double x, double y, double heading,
            double velocityX, double velocityY, double angularVelocity) {
        super(timestamp, x, y, heading);

        this.velocityX = velocityX;
        this.velocityY = velocityY;
        this.angularVelocity = angularVelocity;
    }

    @Override
    public String toString() {
        return "{\"timestamp\": " + timestamp
            + ", \"x\": " + x
            + ", \"y\": " + y
            + ", \"heading\": " + heading
            + ", \"velocity_x\": " + velocityX
            + ", \"velocity_y\": " + velocityY
            + ", \"angular_velocity\": " + angularVelocity
            + "}";
    }
}
