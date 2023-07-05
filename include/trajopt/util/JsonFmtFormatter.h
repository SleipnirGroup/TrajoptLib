// Copyright (c) TrajoptLib contributors

#pragma once

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#define _JSON_FMT_FORMATTER(Type)                                            \
  template <>                                                                \
  struct fmt::formatter<Type> {                                              \
    constexpr auto parse(fmt::format_parse_context& ctx) {                   \
      return ctx.begin();                                                    \
    }                                                                        \
    auto format(const Type& value, fmt::format_context& ctx) const {         \
      return fmt::format_to(ctx.out(), "{}", nlohmann::json(value).dump(2)); \
    }                                                                        \
  };

// avoid newline backslash warning
