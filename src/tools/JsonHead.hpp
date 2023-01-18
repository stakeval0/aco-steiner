#ifndef _BASIC_JSON_
#define _BASIC_JSON_
#include<map>
#include<cstdint>
#include<string>
#include<vector>

namespace nlohmann{
  template<typename, typename> struct adl_serializer;
  template<template<typename U, typename V, typename... Args> class ObjectType =
    std::map,
    template<typename U, typename... Args> class ArrayType = std::vector,
    class StringType = std::string, class BooleanType = bool,
    class NumberIntegerType = std::int64_t,
    class NumberUnsignedType = std::uint64_t,
    class NumberFloatType = double,
    template<typename U> class AllocatorType = std::allocator,
    template<typename T, typename SFINAE = void> class JSONSerializer =
    adl_serializer>
  class basic_json;
  using json=basic_json<>;
}
#endif//_BASIC_JSON_
using json=nlohmann::json;