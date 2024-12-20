// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/vsync_waiter_android.h"

#include <cmath>
#include <utility>

#include "flutter/common/task_runners.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/platform/android/jni_util.h"
#include "flutter/fml/platform/android/scoped_java_ref.h"
#include "flutter/fml/trace_event.h"
#include "impeller/toolkit/android/choreographer.h"

namespace flutter {

static fml::jni::ScopedJavaGlobalRef<jclass>* g_vsync_waiter_class = nullptr;
static jmethodID g_async_wait_for_vsync_method_ = nullptr;
static std::atomic_uint g_refresh_rate_ = 60;

VsyncWaiterAndroid::VsyncWaiterAndroid(const flutter::TaskRunners& task_runners)
    : VsyncWaiter(task_runners) {}

VsyncWaiterAndroid::~VsyncWaiterAndroid() = default;

// |VsyncWaiter|
void VsyncWaiterAndroid::AwaitVSync() {
  const static bool use_choreographer =
      impeller::android::Choreographer::IsAvailableOnPlatform();
  if (use_choreographer) {
    auto* weak_this = new std::weak_ptr<VsyncWaiter>(shared_from_this());
    fml::TaskRunner::RunNowOrPostTask(
        task_runners_.GetUITaskRunner(), [weak_this]() {
          const auto& choreographer =
              impeller::android::Choreographer::GetInstance();
          choreographer.PostFrameCallback([weak_this](auto time) {
            auto time_ns =
                std::chrono::time_point_cast<std::chrono::nanoseconds>(time)
                    .time_since_epoch()
                    .count();
            OnVsyncFromNDK(time_ns, weak_this);
          });
        });
  } else {
    // TODO(99798): Remove it when we drop support for API level < 29 and 32-bit
    // devices.
    auto* weak_this = new std::weak_ptr<VsyncWaiter>(shared_from_this());
    jlong java_baton = reinterpret_cast<jlong>(weak_this);
    task_runners_.GetPlatformTaskRunner()->PostTask([java_baton]() {
      JNIEnv* env = fml::jni::AttachCurrentThread();
      env->CallStaticVoidMethod(g_vsync_waiter_class->obj(),     //
                                g_async_wait_for_vsync_method_,  //
                                java_baton                       //
      );
    });
  }
}

// static
void VsyncWaiterAndroid::OnVsyncFromNDK(int64_t frame_nanos, void* data) {
  auto frame_time = fml::TimePoint::FromEpochDelta(
      fml::TimeDelta::FromNanoseconds(frame_nanos));
  auto now = fml::TimePoint::Now();
  if (frame_time > now) {
    frame_time = now;
  }
  auto target_time = frame_time + fml::TimeDelta::FromNanoseconds(
                                      1000000000.0 / g_refresh_rate_);

  TRACE_EVENT2_INT("flutter", "PlatformVsync", "frame_start_time",
                   frame_time.ToEpochDelta().ToMicroseconds(),
                   "frame_target_time",
                   target_time.ToEpochDelta().ToMicroseconds());

  auto* weak_this = reinterpret_cast<std::weak_ptr<VsyncWaiter>*>(data);
  ConsumePendingCallback(weak_this, frame_time, target_time);
}

// static
void VsyncWaiterAndroid::OnVsyncFromJava(JNIEnv* env,
                                         jclass jcaller,
                                         jlong frameDelayNanos,
                                         jlong refreshPeriodNanos,
                                         jlong java_baton) {
  auto frame_time =
      fml::TimePoint::Now() - fml::TimeDelta::FromNanoseconds(frameDelayNanos);
  auto target_time =
      frame_time + fml::TimeDelta::FromNanoseconds(refreshPeriodNanos);

  TRACE_EVENT2_INT("flutter", "PlatformVsync", "frame_start_time",
                   frame_time.ToEpochDelta().ToMicroseconds(),
                   "frame_target_time",
                   target_time.ToEpochDelta().ToMicroseconds());

  auto* weak_this = reinterpret_cast<std::weak_ptr<VsyncWaiter>*>(java_baton);
  ConsumePendingCallback(weak_this, frame_time, target_time);
}

// static
void VsyncWaiterAndroid::ConsumePendingCallback(
    std::weak_ptr<VsyncWaiter>* weak_this,
    fml::TimePoint frame_start_time,
    fml::TimePoint frame_target_time) {
  auto shared_this = weak_this->lock();
  delete weak_this;

  if (shared_this) {
    shared_this->FireCallback(frame_start_time, frame_target_time);
  }
}

// static
void VsyncWaiterAndroid::OnUpdateRefreshRate(JNIEnv* env,
                                             jclass jcaller,
                                             jfloat refresh_rate) {
  FML_DCHECK(refresh_rate > 0);
  g_refresh_rate_ = static_cast<uint>(refresh_rate);
}

// static
bool VsyncWaiterAndroid::Register(JNIEnv* env) {
  static const JNINativeMethod methods[] = {
      {
          .name = "nativeOnVsync",
          .signature = "(JJJ)V",
          .fnPtr = reinterpret_cast<void*>(&OnVsyncFromJava),
      },
      {
          .name = "nativeUpdateRefreshRate",
          .signature = "(F)V",
          .fnPtr = reinterpret_cast<void*>(&OnUpdateRefreshRate),
      }};

  jclass clazz = env->FindClass("io/flutter/embedding/engine/FlutterJNI");

  if (clazz == nullptr) {
    return false;
  }

  g_vsync_waiter_class = new fml::jni::ScopedJavaGlobalRef<jclass>(env, clazz);
  FML_CHECK(!g_vsync_waiter_class->is_null());

  g_async_wait_for_vsync_method_ = env->GetStaticMethodID(
      g_vsync_waiter_class->obj(), "asyncWaitForVsync", "(J)V");
  FML_CHECK(g_async_wait_for_vsync_method_ != nullptr);

  return env->RegisterNatives(clazz, methods, std::size(methods)) == 0;
}

}  // namespace flutter
