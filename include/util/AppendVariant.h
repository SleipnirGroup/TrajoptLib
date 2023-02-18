// Copyright (c) TrajoptLib contributors

#pragma once

#include <variant>

/**
 * @brief Generates a derived variant type including the types of a
 * base variant and additional types listed. This function should not
 * be evaluated. It may be used along with `decltype()`. For example:
 *
 * ```cpp
 * std::variant<int> myvar; // int
 * decltype(_append_variant(myvar, double())) myvar2; // int, double
 * myvar2 = 1;
 * myvar2 = 2.0;
 * ```
 *
 * @tparam Vs The types of the base variant.
 * @tparam Aps The types to append to the new variant type.
 * @param myvar A variant of the base typ, used only for type deduction.
 * @param newvals values of the new variant
 * @return a default-constructed variant of the derived type.
 */
template <typename... Vs, typename... Aps>
std::variant<Vs..., Aps...> _append_variant(
    [[maybe_unused]] std::variant<Vs...> myvar,
    [[maybe_unused]] Aps... newvals) {
  return std::variant<Vs..., Aps...>();
}
