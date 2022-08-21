package org.team2363.helixtrajectory;

import static org.team2363.util.ObjectChecker.requireNonNullAndWrapUnmodifiable;

import java.util.List;

public final class HolonomicPath extends Path {

    public final List<? extends HolonomicWaypoint> holonomicWaypoints;
    
    @SuppressWarnings("unchecked")
    public HolonomicPath(List<? extends HolonomicWaypoint> holonomicWaypoints) throws NullPointerException {
        super(requireNonNullAndWrapUnmodifiable(holonomicWaypoints,
                "List of holonomic waypoints cannot be null", "Holonomic waypoint cannot be null"));

        this.holonomicWaypoints = (List<? extends HolonomicWaypoint>) waypoints;
    }
}