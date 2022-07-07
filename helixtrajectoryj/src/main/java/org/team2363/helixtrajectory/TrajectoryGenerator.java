package org.team2363.helixtrajectory;

public class TrajectoryGenerator {

    public static Trajectory generate(SwerveDrive drive, Path path, Obstacle[] obstacles) {
        return Trajectory.fromArray(new HelixTrajectoryInterface().generateTrajectory(drive, path, obstacles));
    }
}