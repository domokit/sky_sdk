// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/platform_message_response_android.h"

#include <utility>

#include "flutter/fml/make_copyable.h"
#include "flutter/shell/platform/android/jni/platform_view_android_jni.h"

namespace flutter {

PlatformMessageResponseAndroid::PlatformMessageResponseAndroid(
    int response_id,
    std::shared_ptr<PlatformViewAndroidJNI> jni_facade,
    fml::RefPtr<fml::TaskRunner> platform_task_runner)
    : response_id_(response_id),
      jni_facade_(std::move(jni_facade)),
      platform_task_runner_(std::move(platform_task_runner)) {}

PlatformMessageResponseAndroid::~PlatformMessageResponseAndroid() = default;

// |flutter::PlatformMessageResponse|
void PlatformMessageResponseAndroid::Complete(
    std::unique_ptr<fml::Mapping> data) {
  platform_task_runner_->PostTask(
      fml::MakeCopyable([response_id = response_id_,  //
                         data = std::move(data),      //
                         jni_facade = jni_facade_]() mutable {
        jni_facade->FlutterViewHandlePlatformMessageResponse(response_id,
                                                             std::move(data));
      }));
}

// |flutter::PlatformMessageResponse|
void PlatformMessageResponseAndroid::CompleteEmpty() {
  platform_task_runner_->PostTask(
      fml::MakeCopyable([response_id = response_id_,  //
                         jni_facade = jni_facade_     //
  ]() {
        // Make the response call into Java.
        jni_facade->FlutterViewHandlePlatformMessageResponse(response_id,
                                                             nullptr);
      }));
}
}  // namespace flutter
