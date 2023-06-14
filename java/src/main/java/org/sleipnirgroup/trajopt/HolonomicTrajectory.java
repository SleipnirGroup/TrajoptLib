// Copyright (c) TrajoptLib contributors

package org.sleipnirgroup.trajopt;

import static org.sleipnirgroup.util.ObjectChecker.requireNonNullAndWrapUnmodifiable;

import java.util.List;

public final class HolonomicTrajectory extends Trajectory {
    public final List<? extends HolonomicTrajectorySample> holonomicSamples;

    @SuppressWarnings("unchecked")
    public HolonomicTrajectory(List<? extends HolonomicTrajectorySample> holonomicSamples)
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
