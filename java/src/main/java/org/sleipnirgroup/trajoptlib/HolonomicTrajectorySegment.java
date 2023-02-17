// Copyright (c) TrajoptLib contributors

package org.sleipnirgroup.trajoptlib;

import static org.sleipnirgroup.util.ObjectChecker.requireNonNullAndWrapUnmodifiable;

import java.util.List;

public final class HolonomicTrajectorySegment extends TrajectorySegment {
    public final List<? extends HolonomicTrajectorySample> holonomicSamples;

    @SuppressWarnings("unchecked")
    public HolonomicTrajectorySegment(List<? extends HolonomicTrajectorySample> holonomicSamples)
            throws NullPointerException {
        super(requireNonNullAndWrapUnmodifiable(holonomicSamples,
                "List of holonomic samples cannot be null", "Holonomic sample cannot be null"));

        this.holonomicSamples = (List<? extends HolonomicTrajectorySample>) samples;
    }

    @Override
    public String toString() {
        return holonomicSamples.toString();
    }
}
