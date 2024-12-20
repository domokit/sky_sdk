// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_BASE_TIMING_H_
#define FLUTTER_IMPELLER_BASE_TIMING_H_

#include <chrono>

namespace impeller {

using MillisecondsF = std::chrono::duration<float, std::milli>;
using SecondsF = std::chrono::duration<float>;
using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_BASE_TIMING_H_
