package org.team2363.helixtrajectory;

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
}