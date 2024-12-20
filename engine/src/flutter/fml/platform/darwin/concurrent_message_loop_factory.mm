// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/concurrent_message_loop.h"

namespace fml {

class ConcurrentMessageLoopDarwin : public ConcurrentMessageLoop {
  friend class ConcurrentMessageLoop;

 protected:
  explicit ConcurrentMessageLoopDarwin(size_t worker_count) : ConcurrentMessageLoop(worker_count) {}

  void ExecuteTask(const fml::closure& task) override {
    @autoreleasepool {
      task();
    }
  }
};

std::shared_ptr<ConcurrentMessageLoop> ConcurrentMessageLoop::Create(size_t worker_count) {
  return std::shared_ptr<ConcurrentMessageLoop>{new ConcurrentMessageLoopDarwin(worker_count)};
}

}  // namespace fml
