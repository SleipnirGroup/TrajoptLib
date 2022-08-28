#pragma once

#include <stdexcept>
#include <string>

namespace helixtrajectory {

    /**
     * @brief This exception is thrown when an invalid path is used with a trajectory
     * generator.
     * 
     * @author Justin Babilino
     */
    class InvalidPathException : public std::logic_error {
    public:
        /**
         * @brief Construct a new Invalid Path Exception object with a string message
         * 
         * @param message the message
         */
        explicit InvalidPathException(const std::string& message);
        /**
         * @brief Construct a new Invalid Path Exception object with a string message
         * 
         * @param message the message
         */
        explicit InvalidPathException(const char* message);
    };
}