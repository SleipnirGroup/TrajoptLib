package org.team2363.helixtrajectory;

import java.util.List;

public class SwerveDrivetrain extends HolonomicDrivetrain {

    public final List<SwerveModule> modules;

    public SwerveDrivetrain(double mass, double moi, Obstacle bumpers, SwerveModule... modules) throws NullPointerException {
        super(mass, moi, bumpers);

        this.modules = List.of(modules);
    }
}