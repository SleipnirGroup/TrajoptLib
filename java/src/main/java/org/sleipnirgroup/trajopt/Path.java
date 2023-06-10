// Copyright (c) TrajoptLib contributors

package org.sleipnirgroup.trajopt;

import static org.sleipnirgroup.util.ObjectChecker.requireNonNullAndWrapUnmodifiable;

import java.util.List;

public abstract class Path {
    public final List<? extends Waypoint> waypoints;
    public final List<Obstacle> obstacles;

    protected Path(List<? extends Waypoint> waypoints, List<Obstacle> obstacles) {
        this.waypoints = waypoints;
        this.obstacles = requireNonNullAndWrapUnmodifiable(obstacles,
                "List of obstacles cannot be null", "Obstacle cannot be null");
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
                || !waypoints.get(0).isStateKnown() || !waypoints.get(waypoints.size() - 1).isStateKnown()) {
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
