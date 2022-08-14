package org.team2363.helixtrajectory;

public abstract class HolonomicDrivetrain extends Drivetrain {
    
    protected HolonomicDrivetrain(double mass, double momentOfInertia, Obstacle bumpers) throws NullPointerException {
        super(mass, momentOfInertia, bumpers);
    }
}
