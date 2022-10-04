#pragma once

namespace helixtrajectory {

    class ScalarBound {
    public:
        double lowerBound;
        double upperBound;

        ScalarBound(double lowerBound, double upperBound);

        bool IsValid() const noexcept;
    };
}