// Copyright (c) TrajoptLib contributors

package org.sleipnirgroup.trajoptlib;

import java.util.List;

public abstract class TrajectorySegment {
    public final List<? extends TrajectorySample> samples;

    protected TrajectorySegment(List<? extends TrajectorySample> samples) {
        this.samples = samples;
    }
}
