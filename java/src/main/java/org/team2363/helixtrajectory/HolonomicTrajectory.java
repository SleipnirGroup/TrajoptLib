// Copyright (c) TrajoptLib contributors

package org.team2363.helixtrajectory;

import static org.team2363.util.ObjectChecker.requireNonNullAndWrapUnmodifiable;

import java.util.List;

public final class HolonomicTrajectory extends Trajectory {
    public final List<? extends HolonomicTrajectorySegment> holonomicSegments;

    @SuppressWarnings("unchecked")
    public HolonomicTrajectory(List<? extends HolonomicTrajectorySegment> holonomicSegments) throws NullPointerException {
        super(requireNonNullAndWrapUnmodifiable(holonomicSegments,
                "List of holonomic trajectory segments cannot be null",
                "Holonomic trajectory segment cannot be null"));

        this.holonomicSegments = (List<? extends HolonomicTrajectorySegment>) segments;
    }

    @Override
    public String toString() {
        return holonomicSegments.toString();
    }
}
