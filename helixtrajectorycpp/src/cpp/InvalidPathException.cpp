#include "InvalidPathException.h"

#include <stdexcept>
#include <string>

namespace helixtrajectory {

    InvalidPathException::InvalidPathException(const std::string& message)
            : logic_error(message) {
    }

    InvalidPathException::InvalidPathException(const char* message)
            : logic_error(message) {
    }
}