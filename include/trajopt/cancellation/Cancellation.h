#pragma once
#include <atomic>

namespace trajopt {
    TRAJOPT_DLLEXPORT std::atomic<int>& GetCancellationFlag();
}