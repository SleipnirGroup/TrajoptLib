#pragma once

namespace helixtrajectory {

    class VectorBound {
    public:
        virtual ~VectorBound() = default;

        virtual bool IsValid() const noexcept = 0;
    };
}