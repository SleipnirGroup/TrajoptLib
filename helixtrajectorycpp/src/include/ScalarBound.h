#pragma once

namespace helixtrajectory {

    /**
     * @brief This class represents a bounded region of abstract 1D space. The bound
     * is specified by an inclusive inequality between two abstract coordinates.
     * 
     * @author Justin Babilino
     */
    class ScalarBound {
    public:
        /**
         * @brief the lower bound 
         */
        double lower;
        /**
         * @brief the upper bound
         */
        double upper;

        /**
         * @brief Construct a Scalar Bound between a lower and upper bound.
         * 
         * @param lower the lower bound
         * @param upper the upper bound
         */
        ScalarBound(double lower, double upper);

        /**
         * @brief Check if this scalar bound is valid. A scalar bound is valid
         * if and only if the lower bound is less than or equal to the upper
         * bound.
         * 
         * @return true if and only if this sclar bound is valid
         */
        bool IsValid() const noexcept;
    };
}