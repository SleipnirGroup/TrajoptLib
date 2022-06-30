package org.team2363.helixtrajectory;

public class Path {

    final Waypoint[] waypoints;

    public Path(Waypoint... waypoints) {
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
