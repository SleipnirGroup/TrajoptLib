#pragma once
#include <atomic>

namespace trajopt {
    std::atomic<int>& GetCancellationFlag() {
        static std::atomic<int> flag;
        return flag;
    }
}