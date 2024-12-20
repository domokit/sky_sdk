// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/post_task_sync.h"

#include "flutter/fml/synchronization/waitable_event.h"

namespace flutter::testing {

void PostTaskSync(const fml::RefPtr<fml::TaskRunner>& task_runner,
                  const std::function<void()>& function) {
  fml::AutoResetWaitableEvent latch;
  task_runner->PostTask([&] {
    function();
    latch.Signal();
  });
  latch.Wait();
}

}  // namespace flutter::testing
