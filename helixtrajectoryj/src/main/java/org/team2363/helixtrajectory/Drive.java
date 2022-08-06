package org.team2363.helixtrajectory;

import java.util.Objects;

public abstract class Drive {

    public final double mass;
    public final double moi;
    public final Obstacle bumpers;

    protected Drive(double mass, double moi, Obstacle bumpers) throws NullPointerException {
        this.mass = mass;
        this.moi = moi;
        this.bumpers = Objects.requireNonNull(bumpers, "Drive bumpers cannot be null");
    }
}