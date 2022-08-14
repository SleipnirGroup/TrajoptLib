package org.team2363.helixtrajectory;

import java.util.List;
import java.util.Objects;

public final class OptimalHolonomicTrajectoryGenerator implements HolonomicTrajectoryGenerator {

    static {
        System.loadLibrary("helixtrajectorycpp");
    }

    private final HolonomicDrivetrain holonomicDrive;
    private final HolonomicPath holonomicPath;
    private final List<Obstacle> obstacles;

    public OptimalHolonomicTrajectoryGenerator(HolonomicDrivetrain holonomicDrive, HolonomicPath holonomicPath, Obstacle... obstacles) throws NullPointerException {
        this.holonomicDrive = Objects.requireNonNull(holonomicDrive, "Holonomic Trajectory Generator holonomic drive cannot be null");
        this.holonomicPath = Objects.requireNonNull(holonomicPath, "Holonomic Trajectory Generator holonomic path cannot be null");
        this.obstacles = List.of(obstacles);
    }

    @Override
    public native HolonomicTrajectory generate();
}