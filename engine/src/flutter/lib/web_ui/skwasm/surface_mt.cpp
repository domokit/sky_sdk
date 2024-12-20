// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "surface.h"

#include "skwasm_support.h"

using namespace Skwasm;

Surface::Surface() {
  assert(emscripten_is_main_browser_thread());

  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

  pthread_create(
      &_thread, &attr,
      [](void* context) -> void* {
        static_cast<Surface*>(context)->_runWorker();
        return nullptr;
      },
      this);
  // Listen to messages from the worker
  skwasm_connectThread(_thread);
}

SKWASM_EXPORT bool skwasm_isMultiThreaded() {
  return true;
}
