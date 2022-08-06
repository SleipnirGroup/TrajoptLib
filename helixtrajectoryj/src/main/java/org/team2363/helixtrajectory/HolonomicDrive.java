package org.team2363.helixtrajectory;

public abstract class HolonomicDrive extends Drive {
    
    protected HolonomicDrive(double mass, double moi, Obstacle bumpers) throws NullPointerException {
        super(mass, moi, bumpers);
    }
}
