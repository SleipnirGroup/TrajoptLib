package org.team2363.helixtrajectory;

import java.util.List;

public abstract class Path {

    public final List<? extends Waypoint> waypoints;

    protected Path(List<? extends Waypoint> waypoints) {
        this.waypoints = waypoints;
    }

    public int controlIntervalTotal() {
        int controlIntervalTotal = 0;
        for (int waypointIndex = 1; waypointIndex < waypoints.size(); waypointIndex++) {
            controlIntervalTotal += waypoints.get(waypointIndex).controlIntervalCount;
        }
        return controlIntervalTotal;
    }

    public boolean isValid() {
        if (waypoints.isEmpty() || controlIntervalTotal() == 0
                || !waypoints.get(0).isInitialWaypoint()) {
            return false;
        }
        for (int index = 0; index < waypoints.size(); index++) {
            if (!waypoints.get(index).isValid()) {
                return false;
            }
        }
        return true;
    }
}