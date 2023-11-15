// Copyright (c) TrajoptLib contributors

#include "trajopt/cancellation/Cancellation.h"

#include <atomic>

namespace trajopt {

std::atomic<int>& GetCancellationFlag() {
  static std::atomic<int> flag{0};
  return flag;
}

}  // namespace trajopt
