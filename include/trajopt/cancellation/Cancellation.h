#pragma once
#include <atomic>
#include "trajopt/SymbolExports.h"

namespace trajopt {
    TRAJOPT_DLLEXPORT std::atomic<int>& GetCancellationFlag();
}