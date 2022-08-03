package org.team2363.helixtrajectory;

public class SwerveDrive {

    public final double wheelbaseX, wheelbaseY, mass, moi,
            omegaMax, tauMax, wheelRadius;
    public final Obstacle bumpers;

    public SwerveDrive(double wheelbaseX, double wheelbaseY,
            double mass, double moi,
            double omegaMax, double tauMax, double wheelRadius,
            Obstacle bumpers) {
        this.wheelbaseX = wheelbaseX;
        this.wheelbaseY = wheelbaseY;
        this.mass = mass;
        this.moi = moi;
        this.omegaMax = omegaMax;
        this.tauMax = tauMax;
        this.wheelRadius = wheelRadius;
        this.bumpers = bumpers;
    }
}