// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_PLATFORM_LINUX_MESSAGE_LOOP_LINUX_H_
#define FLUTTER_FML_PLATFORM_LINUX_MESSAGE_LOOP_LINUX_H_

#include <atomic>

#include "flutter/fml/macros.h"
#include "flutter/fml/message_loop_impl.h"
#include "flutter/fml/unique_fd.h"

namespace fml {

class MessageLoopLinux : public MessageLoopImpl {
 private:
  fml::UniqueFD epoll_fd_;
  fml::UniqueFD timer_fd_;
  bool running_ = false;

  MessageLoopLinux();

  ~MessageLoopLinux() override;

  // |fml::MessageLoopImpl|
  void Run() override;

  // |fml::MessageLoopImpl|
  void Terminate() override;

  // |fml::MessageLoopImpl|
  void WakeUp(fml::TimePoint time_point) override;

  void OnEventFired();

  bool AddOrRemoveTimerSource(bool add);

  FML_FRIEND_MAKE_REF_COUNTED(MessageLoopLinux);
  FML_FRIEND_REF_COUNTED_THREAD_SAFE(MessageLoopLinux);
  FML_DISALLOW_COPY_AND_ASSIGN(MessageLoopLinux);
};

}  // namespace fml

#endif  // FLUTTER_FML_PLATFORM_LINUX_MESSAGE_LOOP_LINUX_H_
