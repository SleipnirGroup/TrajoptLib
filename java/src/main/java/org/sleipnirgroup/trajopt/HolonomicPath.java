// Copyright (c) TrajoptLib contributors

package org.sleipnirgroup.trajopt;

import static org.sleipnirgroup.util.ObjectChecker.requireNonNullAndWrapUnmodifiable;

import java.util.List;

public final class HolonomicPath extends Path {
    public final List<? extends HolonomicWaypoint> holonomicWaypoints;

    @SuppressWarnings("unchecked")
    public HolonomicPath(List<? extends HolonomicWaypoint> holonomicWaypoints, List<Obstacle> obstacles) throws NullPointerException {
        super(requireNonNullAndWrapUnmodifiable(holonomicWaypoints,
                "List of holonomic waypoints cannot be null", "Holonomic waypoint cannot be null"),
                obstacles);

        this.holonomicWaypoints = (List<? extends HolonomicWaypoint>) waypoints;
    }

    @Override
    public String toString() {
        return holonomicWaypoints.toString();
    }
}
