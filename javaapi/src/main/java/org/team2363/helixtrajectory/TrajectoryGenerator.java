package org.team2363.helixtrajectory;

public class TrajectoryGenerator {
    
    private final SwerveDrive drive;
    
    public TrajectoryGenerator(SwerveDrive drive) {
        this.drive = drive;
    }

    public Trajectory generate(Path path) {
        return Trajectory.fromArray(new HelixTrajectoryInterface().generateTrajectory(drive.asArray(), path.asArray()));
    }
}