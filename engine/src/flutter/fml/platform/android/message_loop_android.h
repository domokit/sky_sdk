// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_PLATFORM_ANDROID_MESSAGE_LOOP_ANDROID_H_
#define FLUTTER_FML_PLATFORM_ANDROID_MESSAGE_LOOP_ANDROID_H_

#include <android/looper.h>

#include <atomic>

#include "flutter/fml/macros.h"
#include "flutter/fml/message_loop_impl.h"
#include "flutter/fml/unique_fd.h"

namespace fml {

struct UniqueLooperTraits {
  static ALooper* InvalidValue() { return nullptr; }
  static bool IsValid(ALooper* value) { return value != nullptr; }
  static void Free(ALooper* value) { ::ALooper_release(value); }
};

/// Android implementation of \p MessageLoopImpl.
///
/// This implemenation wraps usage of Android's \p looper.
/// \see https://developer.android.com/ndk/reference/group/looper
class MessageLoopAndroid : public MessageLoopImpl {
 private:
  fml::UniqueObject<ALooper*, UniqueLooperTraits> looper_;
  fml::UniqueFD timer_fd_;
  bool running_ = false;

  MessageLoopAndroid();

  ~MessageLoopAndroid() override;

  void Run() override;

  void Terminate() override;

  void WakeUp(fml::TimePoint time_point) override;

  void OnEventFired();

  FML_FRIEND_MAKE_REF_COUNTED(MessageLoopAndroid);
  FML_FRIEND_REF_COUNTED_THREAD_SAFE(MessageLoopAndroid);
  FML_DISALLOW_COPY_AND_ASSIGN(MessageLoopAndroid);
};

}  // namespace fml

#endif  // FLUTTER_FML_PLATFORM_ANDROID_MESSAGE_LOOP_ANDROID_H_
