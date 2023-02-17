// Copyright (c) TrajoptLib contributors

package org.sleipnirgroup.trajopt;

import java.util.List;

public abstract class TrajectorySegment {
    public final List<? extends TrajectorySample> samples;

    protected TrajectorySegment(List<? extends TrajectorySample> samples) {
        this.samples = samples;
    }
}
