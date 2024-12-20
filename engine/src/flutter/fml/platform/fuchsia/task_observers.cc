// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/platform/fuchsia/task_observers.h"

#include <map>

namespace fml {

thread_local std::map<intptr_t, fit::closure> tTaskObservers;

void ExecuteAfterTaskObservers() {
  for (const auto& callback : tTaskObservers) {
    callback.second();
  }
}

void CurrentMessageLoopAddAfterTaskObserver(intptr_t key,
                                            fit::closure observer) {
  if (observer) {
    tTaskObservers[key] = std::move(observer);
  }
}

void CurrentMessageLoopRemoveAfterTaskObserver(intptr_t key) {
  tTaskObservers.erase(key);
}

}  // namespace fml
