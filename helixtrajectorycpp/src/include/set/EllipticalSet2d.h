#pragma once

namespace helixtrajectory {

class EllipticalSet2d {
public:
    enum class Direction {
        kInside,
        kCentered,
        kOutside
    };

    double xRadius;
    double yRadius;
    Direction direction;

    EllipticalSet2d(double xRadius, double yRadius, Direction direction = Direction::kInside);
    static EllipticalSet2d CircularSet2d(double radius, Direction direction = Direction::kInside);

    bool IsCircular() const noexcept;
    bool IsR2() const noexcept;

    template<typename Expression>
    decltype(Expression() == Expression()) GetConstraint(const Expression& x, const Expression& y) const {
        auto scaledVectorXSquared = (x * x) / (xRadius * xRadius);
        auto scaledVectorYSquared = (y * y) / (yRadius * yRadius);
        auto lhs = scaledVectorXSquared + scaledVectorYSquared;
        switch (direction) {
            case Direction::kInside:   return lhs <= 1.0;
            case Direction::kCentered: return lhs == 1.0;
            case Direction::kOutside:  return lhs >= 1.0;
        }
    }

    void CheckVector(double x, double y) const;

    bool IsValid() const noexcept;
};
}