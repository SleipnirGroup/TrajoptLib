#pragma once

#include <optional>
#include <string>

struct SolutionError {
    std::string errorMessage;
};

struct Tolerances {
    double errorMargin;
};