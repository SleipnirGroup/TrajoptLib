#pragma once
#include <atomic>

namespace trajopt {
    std::atomic<int>& GetCancellationFlag();
}