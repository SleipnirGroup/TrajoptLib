package org.team2363.helixtrajectory;

public class Trajectory {

    final TrajectorySample[] waypoints;

    public TrajectorySample(TrajectorySample... waypoints) {
        this.waypoints = waypoints;
    }

    public int length() {
        return waypoints.length;
    }

    public Waypoint get(int index) throws IndexOutOfBoundsException {
        if (index >= 0 && index < length()) {
            return waypoints[index];
        } else {
            throw new IndexOutOfBoundsException();
        }
    }
}
