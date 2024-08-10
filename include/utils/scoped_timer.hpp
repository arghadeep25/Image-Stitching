/**
 * @file scoped_timer.hpp
 * @details Header to get the execution time of a block of code.
 * @author Arghadeep Mazumder
 * @version 0.1.0
 * @copyright -
 */

#ifndef IMAGE_STITCHING_SCOPED_TIMER_HPP
#define IMAGE_STITCHING_SCOPED_TIMER_HPP

#include <chrono>
#include <iostream>

struct ScopedTimer {
  using Clock = std::chrono::high_resolution_clock;
  std::chrono::time_point<Clock> start;
  ScopedTimer() : start(Clock::now()) {}
  ~ScopedTimer() {
    const auto end = Clock::now();
    std::cout << std::chrono::duration<double>(end - start).count() << " seconds" << std::endl;
  }
};

#endif // IMAGE_STITCHING_SCOPED_TIMER_HPP
