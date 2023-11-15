#pragma once
#include <atomic>
#include "trajopt/SymbolExports.h"

namespace trajopt {
    std::atomic<int>& TRAJOPT_DLLEXPOT GetCancellationFlag();
}