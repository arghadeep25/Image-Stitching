//
// Created by arghadeep on 29.07.24.
//

#ifndef IMAGE_STITCHING_INSTRUMENT_HPP
#define IMAGE_STITCHING_INSTRUMENT_HPP

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <list>
#include <numeric>

#include "time_units.hpp"

#define INSTRUMENTING

#ifdef INSTRUMENTING

#define INSTRUMENT_NANOS                                  \
  using Units__ = std::chrono::nanoseconds;               \
  constexpr bool verbose__ = false;                       \
  static Instrumenter<Units__, verbose__> instrumenter__( \
      __func__, __PRETTY_FUNCTION__, __LINE__, __FILE__); \
  const Timer<Units__, verbose__> timer(instrumenter__);

#define INSTRUMENT_MICROS                                 \
  using Units__ = std::chrono::microseconds;              \
  constexpr bool verbose__ = false;                       \
  static Instrumenter<Units__, verbose__> instrumenter__( \
      __func__, __PRETTY_FUNCTION__, __LINE__, __FILE__); \
  const Timer<Units__, verbose__> timer(instrumenter__);

#define INSTRUMENT_MILLIS                                 \
  using Units__ = std::chrono::milliseconds;              \
  constexpr bool verbose__ = false;                       \
  static Instrumenter<Units__, verbose__> instrumenter__( \
      __func__, __PRETTY_FUNCTION__, __LINE__, __FILE__); \
  const Timer<Units__, verbose__> timer(instrumenter__);

#define INSTRUMENT_NANOS_V                                \
  using Units__ = std::chrono::nanoseconds;               \
  constexpr bool verbose__ = true;                        \
  static Instrumenter<Units__, verbose__> instrumenter__( \
      __func__, __PRETTY_FUNCTION__, __LINE__, __FILE__); \
  const Timer<Units__, verbose__> timer(instrumenter__);

#define INSTRUMENT_MICROS_V                               \
  using Units__ = std::chrono::microseconds;              \
  constexpr bool verbose__ = true;                        \
  static Instrumenter<Units__, verbose__> instrumenter__( \
      __func__, __PRETTY_FUNCTION__, __LINE__, __FILE__); \
  const Timer<Units__, verbose__> timer(instrumenter__);

#define INSTRUMENT_MILLIS_V                               \
  using Units__ = std::chrono::milliseconds;              \
  constexpr bool verbose__ = true;                        \
  static Instrumenter<Units__, verbose__> instrumenter__( \
      __func__, __PRETTY_FUNCTION__, __LINE__, __FILE__); \
  const Timer<Units__, verbose__> timer(instrumenter__);

#else

#define INSTRUMENT_NANOS
#define INSTRUMENT_MICROS
#define INSTRUMENT_MILLIS
#define INSTRUMENT_NANOS_V
#define INSTRUMENT_MICROS_V
#define INSTRUMENT_MILLIS_V

#endif

#define INSTRUMENT INSTRUMENT_MILLIS
#define INSTRUMENT_V INSTRUMENT_MICROS_V

template <typename Units = std::chrono::microseconds, bool verbose = false>
struct Instrumenter {
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint = std::chrono::time_point<Clock>;
  using Time = decltype(std::declval<TimePoint>() - std::declval<TimePoint>());

  static constexpr auto BR = "\n---------------------\n";
  static constexpr auto UNITS = time_units::name<Units>();

  const char* const name_;
  const char* const fullname_;
  size_t line_;
  const char* const file_;
  std::list<size_t> times_;

  Instrumenter(const char* name, const char* fullname, size_t line,
               const char* file)
      : name_(name), fullname_(fullname), line_(line), file_(file) {
    if (verbose) {
      std::cout << BR << "\nInstrumenting: " << name_ << " on line " << line_
                << " in " << file_ << "\n";
      std::cout << fullname << "\n" << BR;
    }
  }

  template <typename T>
  static auto median(const T& t) {
    return *std::next(t.begin(), t.size() / 2);
  }

  template <typename T>
  static auto total(const T& t) {
    return std::accumulate(t.begin(), t.end(), (typename T::value_type)0);
  }

  template <typename T>
  static constexpr auto seconds(T t) {
    return (double)t / Units::period::den;
  }

  ~Instrumenter() {
    std::cout << BR << "\nInstrumentation report for '" << name_ << "' on line "
              << line_ << " in " << file_ << "\n\n";
    std::cout << fullname_ << "\n\n";
    std::cout << "function calls : " << times_.size() << "\n";
    if (verbose) {
      std::cout << "Unsorted times :";
      for (auto time : times_) {
        std::cout << std::fixed << std::showpoint << std::setw(9)
                  << std::setprecision(6) << " " << seconds(time) << ",";
      }
      std::cout << "\n";
    }
    times_.sort();

    std::cout << std::fixed << std::showpoint << std::setw(9)
              << std::setprecision(6)
              << "min    time    : " << seconds(times_.front()) << " seconds\n"
              << "max    time    : " << seconds(times_.back()) << " seconds\n"
              << "median time    : " << seconds(median(times_)) << " seconds\n"
              << "total  time    : " << seconds(total(times_)) << " seconds\n";
  }

  void insert(size_t time) { times_.push_back(time); }
};

template <typename Units, bool verbose>
struct Timer {
  using Clock = typename Instrumenter<Units, verbose>::Clock;

  std::chrono::time_point<Clock> start;
  Instrumenter<Units, verbose>& instrumenter;

  Timer(Instrumenter<Units, verbose>& instrumenter)
      : start(Clock::now()), instrumenter(instrumenter) {}

  ~Timer() {
    const auto end = Clock::now();
    instrumenter.insert(std::chrono::duration_cast<Units>(end - start).count());
  }
};

#endif // IMAGE_STITCHING_INSTRUMENT_HPP
