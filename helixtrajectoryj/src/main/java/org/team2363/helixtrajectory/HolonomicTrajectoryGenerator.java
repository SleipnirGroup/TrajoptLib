package org.team2363.helixtrajectory;

import java.util.Collection;
import java.util.List;
import java.util.Objects;

public final class HolonomicTrajectoryGenerator {

    static {
        System.loadLibrary("helixtrajectorycpp");
    }

    private final HolonomicDrive holonomicDrive;
    private final HolonomicPath holonomicPath;
    private final Collection<Obstacle> obstacles;

    public HolonomicTrajectoryGenerator(HolonomicDrive holonomicDrive, HolonomicPath holonomicPath, Obstacle... obstacles) throws NullPointerException {
        this.holonomicDrive = Objects.requireNonNull(holonomicDrive, "Holonomic Trajectory Generator holonomic drive cannot be null");
        this.holonomicPath = Objects.requireNonNull(holonomicPath, "Holonomic Trajectory Generator holonomic path cannot be null");
        this.obstacles = List.of(obstacles);
    }

    public native HolonomicTrajectory generate();
}