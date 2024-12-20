// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "third_party/benchmark/include/benchmark/benchmark.h"

#include "library.h"

extern "C" {

int RunBenchmarks(int argc, char** argv) {
  benchmark::Initialize(&argc, argv);
  ::benchmark::RunSpecifiedBenchmarks();
  return 0;
}
}
