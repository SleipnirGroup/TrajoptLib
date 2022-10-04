#pragma once

#include "VectorBound.h"

namespace helixtrajectory {

    class RectangularVectorBound : public VectorBound {
    public:
        double leftBound;
        double rightBound;
        double lowerBound;
        double upperBound;

        RectangularVectorBound(double leftBound, double rightBound, double lowerBound, double upperBound);

        virtual bool IsValid() const noexcept override;
    };
}