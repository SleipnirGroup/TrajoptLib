package org.team2363.helixtrajectory;

import java.util.List;

public abstract class Path {

    public final List<? extends Waypoint> waypoints;

    protected Path(List<? extends Waypoint> waypoints) {
        this.waypoints = waypoints;
    }

    public boolean isValid() {
        if (waypoints.isEmpty()) {
            return true;
        }
        if (waypoints.get(0).controlIntervalCount != 0) {
            return false;
        }
        for (int index = 1; index < waypoints.size(); index++) {
            if (waypoints.get(index).controlIntervalCount <= 0) {
                return false;
            }
        }
        return true;
    }
}