// Copyright (c) TrajoptLib contributors

#pragma once

#include <atomic>

namespace trajopt {

std::atomic<int>& GetCancellationFlag();

}  // namespace trajopt
