    #include <atomic>
    #include "optimization/Cancellation.h"
    namespace trajopt {
    std::atomic<int>& GetCancellationFlag() {
        static std::atomic<int> flag;
        return flag;
    }
    }