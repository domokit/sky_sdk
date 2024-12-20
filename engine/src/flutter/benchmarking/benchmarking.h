// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_BENCHMARKING_BENCHMARKING_H_
#define FLUTTER_BENCHMARKING_BENCHMARKING_H_

#include "benchmark/benchmark.h"

namespace benchmarking {

class ScopedPauseTiming {
 public:
  explicit ScopedPauseTiming(::benchmark::State& state, bool enabled = true)
      : state_(state), enabled_(enabled) {
    if (enabled_) {
      state_.PauseTiming();
    }
  }
  ~ScopedPauseTiming() {
    if (enabled_) {
      state_.ResumeTiming();
    }
  }

 private:
  ::benchmark::State& state_;
  const bool enabled_;
};

}  // namespace benchmarking

#endif  // FLUTTER_BENCHMARKING_BENCHMARKING_H_
