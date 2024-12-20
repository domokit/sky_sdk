// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef LIB_TONIC_DART_MICROTASK_QUEUE_H_
#define LIB_TONIC_DART_MICROTASK_QUEUE_H_

#include <vector>

#include "third_party/dart/runtime/include/dart_api.h"
#include "tonic/dart_persistent_value.h"
#include "tonic/logging/dart_error.h"

namespace tonic {

class DartMicrotaskQueue {
 public:
  DartMicrotaskQueue();
  ~DartMicrotaskQueue();

  static void StartForCurrentThread();

  static DartMicrotaskQueue* GetForCurrentThread();

  void ScheduleMicrotask(Dart_Handle callback);
  void RunMicrotasks();
  void Destroy();

  bool HasMicrotasks() const { return !queue_.empty(); }

  DartErrorHandleType GetLastError();

 private:
  typedef std::vector<DartPersistentValue> MicrotaskQueue;

  DartErrorHandleType last_error_;
  MicrotaskQueue queue_;
};

}  // namespace tonic

#endif  // LIB_TONIC_DART_MICROTASK_QUEUE_H_
