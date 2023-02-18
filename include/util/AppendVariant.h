// Copyright (c) TrajoptLib contributors

#pragma once

#include <variant>

// NOLINTBEGIN
template <typename... Vs, typename... Aps>
std::variant<Vs..., Aps...> _append_variant(std::variant<Vs...> myvar,
                                            Aps... newvals) {
  return std::variant<Vs..., Aps...>();
}
// NOLINTEND
