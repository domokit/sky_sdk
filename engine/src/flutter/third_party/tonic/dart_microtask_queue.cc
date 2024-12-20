// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "tonic/dart_microtask_queue.h"

#include "tonic/common/build_config.h"
#include "tonic/dart_state.h"
#include "tonic/logging/dart_invoke.h"

#ifdef OS_IOS
#include <pthread.h>
#endif  // OS_IOS

namespace tonic {
namespace {

#ifdef OS_IOS
// iOS doesn't support the thread_local keyword.

pthread_key_t g_queue_key;
pthread_once_t g_queue_key_once = PTHREAD_ONCE_INIT;

void MakeKey() {
  pthread_key_create(&g_queue_key, nullptr);
}

void SetQueue(DartMicrotaskQueue* queue) {
  pthread_once(&g_queue_key_once, MakeKey);
  pthread_setspecific(g_queue_key, queue);
}

DartMicrotaskQueue* GetQueue() {
  return static_cast<tonic::DartMicrotaskQueue*>(
      pthread_getspecific(g_queue_key));
}

#else

thread_local DartMicrotaskQueue* g_queue = nullptr;

void SetQueue(DartMicrotaskQueue* queue) {
  g_queue = queue;
}

DartMicrotaskQueue* GetQueue() {
  return g_queue;
}

#endif

}  // namespace

DartMicrotaskQueue::DartMicrotaskQueue() : last_error_(kNoError) {}

DartMicrotaskQueue::~DartMicrotaskQueue() = default;

void DartMicrotaskQueue::StartForCurrentThread() {
  SetQueue(new DartMicrotaskQueue());
}

DartMicrotaskQueue* DartMicrotaskQueue::GetForCurrentThread() {
  return GetQueue();
}

void DartMicrotaskQueue::ScheduleMicrotask(Dart_Handle callback) {
  queue_.emplace_back(DartState::Current(), callback);
}

void DartMicrotaskQueue::RunMicrotasks() {
  while (!queue_.empty()) {
    MicrotaskQueue local;
    std::swap(queue_, local);
    for (const auto& callback : local) {
      if (auto dart_state = callback.dart_state().lock()) {
        DartState::Scope dart_scope(dart_state.get());
        Dart_Handle result = Dart_InvokeClosure(callback.value(), 0, nullptr);
        // If the Dart program has set a return code, then it is intending to
        // shut down by way of a fatal error, and so there is no need to emit a
        // log message.
        if (!dart_state->has_set_return_code() || !Dart_IsError(result) ||
            !Dart_IsFatalError(result)) {
          CheckAndHandleError(result);
        }
        DartErrorHandleType error = GetErrorHandleType(result);
        if (error != kNoError) {
          last_error_ = error;
        }
        dart_state->MessageEpilogue(result);
        if (!Dart_CurrentIsolate())
          return;
      }
    }
  }
}

void DartMicrotaskQueue::Destroy() {
  TONIC_DCHECK(this == GetForCurrentThread());
  SetQueue(nullptr);
  delete this;
}

DartErrorHandleType DartMicrotaskQueue::GetLastError() {
  return last_error_;
}

}  // namespace tonic
