// Copyright (c) TrajoptLib contributors

#pragma once

#include <variant>

namespace trajopt {

namespace detail {

//! @cond Doxygen_Suppress

template <typename Variant, typename... Args>
struct variant_cat;

template <typename... Args0, typename... Args1>
struct variant_cat<std::variant<Args0...>, Args1...> {
  using type = std::variant<Args0..., Args1...>;
};

//! @endcond

}  // namespace detail

/**
 * Defines a new variant type that appends the types Args... to the variant type
 * Variant.
 */
template <typename Variant, typename... Args>
using variant_cat_t = typename detail::variant_cat<Variant, Args...>::type;

}  // namespace trajopt
