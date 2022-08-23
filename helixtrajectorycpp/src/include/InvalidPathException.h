#pragma once

#include <stdexcept>
#include <string>

namespace helixtrajectory {

    class InvalidPathException : public std::logic_error {
    public:
        explicit InvalidPathException(const std::string& message);
        explicit InvalidPathException(const char* message);
    };
}