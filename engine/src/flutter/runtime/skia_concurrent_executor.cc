// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/runtime/skia_concurrent_executor.h"

#include "flutter/fml/trace_event.h"

namespace flutter {

SkiaConcurrentExecutor::SkiaConcurrentExecutor(const OnWorkCallback& on_work)
    : on_work_(on_work) {}

SkiaConcurrentExecutor::~SkiaConcurrentExecutor() = default;

void SkiaConcurrentExecutor::add(fml::closure work) {
  if (!work) {
    return;
  }
  on_work_([work]() {
    TRACE_EVENT0("flutter", "SkiaExecutor");
    work();
  });
}

}  // namespace flutter
