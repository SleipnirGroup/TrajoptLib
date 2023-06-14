// Copyright (c) TrajoptLib contributors

package org.sleipnirgroup.trajopt;

public abstract class HolonomicDrivetrain extends Drivetrain {
    protected HolonomicDrivetrain(double mass, double momentOfInertia, Obstacle bumpers) throws NullPointerException {
        super(mass, momentOfInertia, bumpers);
    }
}
