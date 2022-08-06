package org.team2363.helixtrajectory;

import java.util.Collections;
import java.util.List;

public class SwerveDrive extends HolonomicDrive {

    public final List<SwerveModule> modules;

    public SwerveDrive(double mass, double moi, Obstacle bumpers, SwerveModule... modules) throws NullPointerException {
        super(mass, moi, bumpers);

        this.modules = Collections.unmodifiableList(List.of(modules));
    }
}