#pragma once

#include "VectorBound.h"

namespace helixtrajectory {

    class PolarVectorBound : public VectorBound {
    public:
        double magnitudeLowerBound;
        double magnitudeUpperBound;
        double angleLowerBound;
        double angleUpperBound;

        PolarVectorBound(double magnitudeLowerBound, double magnitudeUpperBound, double angleLowerBound, double angleUpperBound);

        virtual bool IsValid() const noexcept override;
    };
}