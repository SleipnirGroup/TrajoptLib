package org.team2363.helixtrajectory;

class HelixTrajectoryInterface {

    static {
        System.loadLibrary("helixtrajectorycpp");
    }

    public native double[] generateTrajectory(SwerveDrive drive, Path path, Obstacle[] obstacles);
}