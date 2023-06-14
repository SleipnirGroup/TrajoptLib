// Copyright (c) TrajoptLib contributors

package org.sleipnirgroup.trajopt;

import java.util.Objects;

public abstract class Drivetrain {
    public final double mass;
    public final double momentOfInertia;
    public final Obstacle bumpers;

    protected Drivetrain(double mass, double momentOfInertia, Obstacle bumpers) throws NullPointerException {
        this.mass = mass;
        this.momentOfInertia = momentOfInertia;
        this.bumpers = Objects.requireNonNull(bumpers, "Drivetrain bumpers cannot be null");
    }
}
