#pragma once

#include <tuple>

// helper template to get the index of a type in a tuple at compile time
// See https://devblogs.microsoft.com/oldnewthing/20200629-00/?p=103910
// usage:
// constexpr std::size_t index = tuple_element_index_v<int, std::tuple<char, int, float>>;  // = 1


template<typename T, typename Tuple>
struct tuple_element_index_helper;

template<typename T>
struct tuple_element_index_helper<T, std::tuple<>>
{
  static constexpr std::size_t value = 0;
};

template<typename T, typename... Rest>
struct tuple_element_index_helper<T, std::tuple<T, Rest...>>
{
  static constexpr std::size_t value = 0;
  using RestTuple = std::tuple<Rest...>;
  static_assert(
    tuple_element_index_helper<T, RestTuple>::value == 
    std::tuple_size_v<RestTuple>,
    "type appears more than once in tuple");
};

template<typename T, typename First, typename... Rest>
struct tuple_element_index_helper<T, std::tuple<First, Rest...>>
{
  using RestTuple = std::tuple<Rest...>;
  static constexpr std::size_t value = 1 +
       tuple_element_index_helper<T, RestTuple>::value;
};

template<typename T, typename Tuple>
struct tuple_element_index
{
  static constexpr std::size_t value =
    tuple_element_index_helper<T, Tuple>::value;
  static_assert(value < std::tuple_size_v<Tuple>,
                "type does not appear in tuple");
};

template<typename T, typename Tuple>
inline constexpr std::size_t tuple_element_index_v
 = tuple_element_index<T, Tuple>::value;
