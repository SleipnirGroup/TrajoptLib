    #include <atomic>
    #include "optimization/Cancellation.h"
    std::atomic<int>& GetCancellationFlag() {
        static std::atomic<int> flag;
        return flag;
    }