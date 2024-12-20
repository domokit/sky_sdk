// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_PLATFORM_WIN_MESSAGE_LOOP_WIN_H_
#define FLUTTER_FML_PLATFORM_WIN_MESSAGE_LOOP_WIN_H_

#include <windows.h>

#include <atomic>

#include "flutter/fml/macros.h"
#include "flutter/fml/message_loop_impl.h"
#include "flutter/fml/unique_object.h"

namespace fml {

class MessageLoopWin : public MessageLoopImpl {
 private:
  struct UniqueHandleTraits {
    static HANDLE InvalidValue() { return NULL; }
    static bool IsValid(HANDLE value) { return value != NULL; }
    static void Free(HANDLE value) { CloseHandle(value); }
  };

  bool running_;
  fml::UniqueObject<HANDLE, UniqueHandleTraits> timer_;
  uint32_t timer_resolution_;

  MessageLoopWin();

  ~MessageLoopWin() override;

  void Run() override;

  void Terminate() override;

  void WakeUp(fml::TimePoint time_point) override;

  FML_FRIEND_MAKE_REF_COUNTED(MessageLoopWin);
  FML_FRIEND_REF_COUNTED_THREAD_SAFE(MessageLoopWin);
  FML_DISALLOW_COPY_AND_ASSIGN(MessageLoopWin);
};

}  // namespace fml

#endif  // FLUTTER_FML_PLATFORM_WIN_MESSAGE_LOOP_WIN_H_
