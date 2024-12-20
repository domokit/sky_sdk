// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/platform_message_handler_android.h"

namespace flutter {

PlatformMessageHandlerAndroid::PlatformMessageHandlerAndroid(
    const std::shared_ptr<PlatformViewAndroidJNI>& jni_facade)
    : jni_facade_(jni_facade) {}

void PlatformMessageHandlerAndroid::InvokePlatformMessageResponseCallback(
    int response_id,
    std::unique_ptr<fml::Mapping> mapping) {
  // Called from any thread.
  if (!response_id) {
    return;
  }
  // TODO(gaaclarke): Move the jump to the ui thread here from
  // PlatformMessageResponseDart so we won't need to use a mutex anymore.
  fml::RefPtr<flutter::PlatformMessageResponse> message_response;
  {
    std::lock_guard lock(pending_responses_mutex_);
    auto it = pending_responses_.find(response_id);
    if (it == pending_responses_.end()) {
      return;
    }
    message_response = std::move(it->second);
    pending_responses_.erase(it);
  }

  message_response->Complete(std::move(mapping));
}

void PlatformMessageHandlerAndroid::InvokePlatformMessageEmptyResponseCallback(
    int response_id) {
  // Called from any thread.
  if (!response_id) {
    return;
  }
  fml::RefPtr<flutter::PlatformMessageResponse> message_response;
  {
    std::lock_guard lock(pending_responses_mutex_);
    auto it = pending_responses_.find(response_id);
    if (it == pending_responses_.end()) {
      return;
    }
    message_response = std::move(it->second);
    pending_responses_.erase(it);
  }
  message_response->CompleteEmpty();
}

// |PlatformView|
void PlatformMessageHandlerAndroid::HandlePlatformMessage(
    std::unique_ptr<flutter::PlatformMessage> message) {
  // Called from any thread.
  int response_id = next_response_id_.fetch_add(1);
  if (auto response = message->response()) {
    std::lock_guard lock(pending_responses_mutex_);
    pending_responses_[response_id] = response;
  }
  // This call can re-enter in InvokePlatformMessageXxxResponseCallback.
  jni_facade_->FlutterViewHandlePlatformMessage(std::move(message),
                                                response_id);
}

}  // namespace flutter
