// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/synchronization/count_down_latch.h"

#include "flutter/fml/logging.h"

namespace fml {

CountDownLatch::CountDownLatch(size_t count) : count_(count) {
  if (count_ == 0) {
    waitable_event_.Signal();
  }
}

CountDownLatch::~CountDownLatch() = default;

void CountDownLatch::Wait() {
  waitable_event_.Wait();
}

void CountDownLatch::CountDown() {
  if (--count_ == 0) {
    waitable_event_.Signal();
  }
}

}  // namespace fml
