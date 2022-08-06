package org.team2363.helixtrajectory;

import java.util.Collections;
import java.util.List;

public abstract class Path {

    public final List<? extends Waypoint> waypoints;

    protected Path(List<? extends Waypoint> waypoints) throws NullPointerException {
        this.waypoints = Collections.unmodifiableList(waypoints);
    }
}
