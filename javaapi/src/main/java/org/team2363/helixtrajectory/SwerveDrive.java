package org.team2363.helixtrajectory;

public class SwerveDrive {
    public final double wheelbaseX, wheelbaseY, length, width, mass, moi,
            omegaMax, tauMax, wheelRadius;
    public SwerveDrive(double wheelbaseX, double wheelbaseY,
            double length, double width, double mass, double moi,
            double omegaMax, double tauMax, double wheelRadius) {
        this.wheelbaseX = wheelbaseX;
        this.wheelbaseY = wheelbaseY;
        this.length = length;
        this.width = width;
        this.mass = mass;
        this.moi = moi;
        this.omegaMax = omegaMax;
        this.tauMax = tauMax;
        this.wheelRadius = wheelRadius;
    }

    double[] asArray() {
        return new double[]{ wheelbaseX, wheelbaseY, length, width, mass, moi,
            omegaMax, tauMax, wheelRadius};
    }
}