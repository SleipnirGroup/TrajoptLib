#pragma once

#include <variant>

template<typename... Vs, typename... Aps>
std::variant<Vs..., Aps...> appendVariant(std::variant<Vs...> myvar, Aps... newvals) {
    return std::variant<Vs..., Aps...>();
}
