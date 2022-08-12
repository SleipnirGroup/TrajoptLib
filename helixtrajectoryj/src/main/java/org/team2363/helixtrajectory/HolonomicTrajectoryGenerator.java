package org.team2363.helixtrajectory;

public interface HolonomicTrajectoryGenerator {
    
    HolonomicTrajectory generate() throws InvalidPathException, TrajectoryGenerationException;

    static boolean checkHolonomicPath(HolonomicPath path) {
        if (path.waypoints.isEmpty()) {
            return true;
        }
        if (path.waypoints.get(0).controlIntervalCount != 0) {
            return false;
        }
        for (int index = 1; index < path.waypoints.size(); index++) {
            if (path.waypoints.get(index).controlIntervalCount <= 0) {
                return false;
            }
        }
        return true;
    }
}