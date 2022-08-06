package org.team2363.helixtrajectory;

import java.util.List;

public final class HolonomicPath extends Path {

    public final List<? extends HolonomicWaypoint> holonomicWaypoints;
    
    private HolonomicPath(List<? extends HolonomicWaypoint> holonomicWaypoints) throws NullPointerException {
        super(holonomicWaypoints);

        this.holonomicWaypoints = holonomicWaypoints;
    }

    public HolonomicPath(HolonomicWaypoint... holonomicWaypoints) {
        this(List.of(holonomicWaypoints));
    }
}