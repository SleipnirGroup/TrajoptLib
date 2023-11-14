    #include <atomic>
    #include "optimization/Cancellation.h"
    namespace trajopt {
    std::atomic<int>& GetCancellationFlag() {
        static std::atomic<int> flag(0);
        return flag;
    }
    }