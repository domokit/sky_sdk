// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_PLATFORM_MESSAGE_HANDLER_ANDROID_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_PLATFORM_MESSAGE_HANDLER_ANDROID_H_

#include <jni.h>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "flutter/lib/ui/window/platform_message.h"
#include "flutter/shell/common/platform_message_handler.h"
#include "flutter/shell/platform/android/jni/platform_view_android_jni.h"

namespace flutter {
class PlatformMessageHandlerAndroid : public PlatformMessageHandler {
 public:
  explicit PlatformMessageHandlerAndroid(
      const std::shared_ptr<PlatformViewAndroidJNI>& jni_facade);
  void HandlePlatformMessage(std::unique_ptr<PlatformMessage> message) override;
  bool DoesHandlePlatformMessageOnPlatformThread() const override {
    return false;
  }
  void InvokePlatformMessageResponseCallback(
      int response_id,
      std::unique_ptr<fml::Mapping> mapping) override;

  void InvokePlatformMessageEmptyResponseCallback(int response_id) override;

 private:
  const std::shared_ptr<PlatformViewAndroidJNI> jni_facade_;
  std::atomic<int> next_response_id_ = 1;
  std::unordered_map<int, fml::RefPtr<flutter::PlatformMessageResponse>>
      pending_responses_;
  std::mutex pending_responses_mutex_;
};
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_PLATFORM_MESSAGE_HANDLER_ANDROID_H_
