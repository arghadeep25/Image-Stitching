//
// Created by arghadeep on 29.07.24.
//

#ifndef IMAGE_STITCHING_TIME_UNITS_HPP
#define IMAGE_STITCHING_TIME_UNITS_HPP

#include <chrono>

namespace time_units {

template <typename TimeUnits = std::chrono::microseconds>
constexpr auto name() {
  return "microseconds";
}

template <>
constexpr auto name<std::chrono::hours>() {
  return "hours";
}
template <>
constexpr auto name<std::chrono::minutes>() {
  return "minutes";
}
template <>
constexpr auto name<std::chrono::seconds>() {
  return "seconds";
}
template <>
constexpr auto name<std::chrono::milliseconds>() {
  return "milliseconds";
}
template <>
constexpr auto name<std::chrono::nanoseconds>() {
  return "nanoseconds";
}

}  // namespace time_units


#endif // IMAGE_STITCHING_TIME_UNITS_HPP
