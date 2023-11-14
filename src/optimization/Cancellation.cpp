    #include <atomic>
    std::atomic<int>& GetCancellationFlag() {
        static std::atomic<int> flag;
        return flag;
    }