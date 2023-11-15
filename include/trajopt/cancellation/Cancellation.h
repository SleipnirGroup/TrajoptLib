#pragma once
#include <atomic>
#include "trajopt/SymbolExports.h"

namespace trajopt {
    TRAJOPT_DLLEXPOT std::atomic<int>& GetCancellationFlag();
}