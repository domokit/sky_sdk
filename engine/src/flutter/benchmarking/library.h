// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_BENCHMARKING_LIBRARY_H_
#define FLUTTER_BENCHMARKING_LIBRARY_H_

extern "C" {
__attribute__((visibility("default"))) int RunBenchmarks(int argc, char** argv);
}

#endif  // FLUTTER_BENCHMARKING_LIBRARY_H_
