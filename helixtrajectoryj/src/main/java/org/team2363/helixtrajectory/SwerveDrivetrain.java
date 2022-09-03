package org.team2363.helixtrajectory;

import static org.team2363.util.ObjectChecker.requireNonNullAndWrapUnmodifiable;

import java.util.List;

public class SwerveDrivetrain extends HolonomicDrivetrain {

    public final List<? extends SwerveModule> modules;

    public SwerveDrivetrain(double mass, double momentOfInertia, List<? extends SwerveModule> modules, Obstacle bumpers) throws NullPointerException {
        super(mass, momentOfInertia, bumpers);

        this.modules = requireNonNullAndWrapUnmodifiable(modules, "List of swerve modules cannot be null", "Swerve module cannot be null");
    }

    @Override
    public String toString() {
        return "{\"mass\": " + mass
            + ", \"moment_of_inertia\": " + momentOfInertia
            + ", \"modules\": " + modules
            + ", \"bumpers\": " + bumpers
            + "}";
    }
}