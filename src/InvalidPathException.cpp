// Copyright (c) TrajoptLib contributors

#include "trajopt/InvalidPathException.h"

namespace trajopt {

InvalidPathException::InvalidPathException(const std::string& message)
    : std::logic_error(message) {}

InvalidPathException::InvalidPathException(const char* message)
    : std::logic_error(message) {}
}  // namespace trajopt
