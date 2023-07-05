#pragma once

#include <nlohmann/json.hpp>

#include <fmt/format.h>

#define _JSON_FMT_FORMATTER(Type)                                             \
template <>                                                                   \
struct fmt::formatter<Type> {                                                 \
  constexpr auto parse(fmt::format_parse_context& ctx) {                      \
    return ctx.begin();                                                       \
  }                                                                           \
  auto format(const Type& value,                                              \
              fmt::format_context& ctx) const {                               \
    return fmt::format_to(ctx.out(),                                          \
        "{}", nlohmann::json(value).dump(2));                                 \
  }                                                                           \
};

// avoid newline backslash warning
