#include "InitialGuessPoint.h"

#include <iostream>

namespace helixtrajectory {

    std::ostream& operator<<(std::ostream& stream, const InitialGuessPoint& guessPoint) {
        return stream << "{\"x\": " << guessPoint.x
                << ", \"y\": " << guessPoint.y
                << ", \"heading\": " << guessPoint.heading
                << "}";
    }
}