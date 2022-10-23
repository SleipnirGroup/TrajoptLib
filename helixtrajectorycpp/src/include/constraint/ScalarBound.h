#pragma once

namespace helixtrajectory {

    /**
     * @brief This class represents a bounded region of abstract 1D space. The bound
     * is specified by an inclusive inequality between two numbers.
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
         * @brief Construct a Scalar Bound that represents the interval
         * [value, value].
         * 
         * @param value the value to bound the number between
         */
        ScalarBound(double value);
        /**
         * @brief Construct Scalar Bound including all numbers from
         * negative infinity to positive infinity.
         */
        ScalarBound();

        /**
         * @brief Check if this scalar bound is equivalent to another scalar bound.
         * Two scalar bounds are equivalent if their upper and lower bounds are equal.
         * This operator can also be used to check if this bound is equal to a number,
         * meaning the upper and lower bounds both equal the given number.
         * 
         * @param other the other scalar bound
         * @return lower == other.lower && upper == other.upper
         */
        bool operator==(const ScalarBound& other) const noexcept;

        /**
         * @brief Calculate the range of this scalar bound, which is the difference
         * between the upper and lower bounds. An infinite sclar bound has a range
         * of positive infinity.
         * 
         * @return upper - lower
         */
        double Range() const noexcept;

        /**
         * @brief Check if this scalar bound only contains one point. This only
         * occurs when lower == upper.
         * 
         * @return lower == upper
         */
        bool IsExact() const noexcept;
        /**
         * @brief Check if this scalar bound only contains 0. This occurs when
         * this scalar bound equals 0.0.
         * 
         * @return lower == 0.0 && upper == 0.0
         */
        bool IsZero() const noexcept;
        /**
         * @brief Check if this scalar bound is infinite. A scalar bound is
         * infinite if contains all numbers.
         * 
         * @return lower <= -inf && upper >= inf
         */
        bool IsInfinite() const noexcept;

        /**
         * @brief Check if this scalar bound is valid. A scalar bound is valid
         * if and only if the lower bound is less than or equal to the upper
         * bound.
         * 
         * @return lower <= upper
         */
        bool IsValid() const noexcept;
    };
}