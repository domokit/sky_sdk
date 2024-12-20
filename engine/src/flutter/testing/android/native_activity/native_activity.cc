// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/android/native_activity/native_activity.h"

#include "flutter/fml/message_loop.h"

namespace flutter {

NativeActivity::NativeActivity(ANativeActivity* activity)
    : activity_(activity) {
  fml::MessageLoop::EnsureInitializedForCurrentThread();

  activity->instance = this;

  activity->callbacks->onStart = [](ANativeActivity* activity) {
    reinterpret_cast<NativeActivity*>(activity->instance)->OnStart();
  };
  activity->callbacks->onStop = [](ANativeActivity* activity) {
    reinterpret_cast<NativeActivity*>(activity->instance)->OnStop();
  };
  activity->callbacks->onPause = [](ANativeActivity* activity) {
    reinterpret_cast<NativeActivity*>(activity->instance)->OnPause();
  };
  activity->callbacks->onResume = [](ANativeActivity* activity) {
    reinterpret_cast<NativeActivity*>(activity->instance)->OnResume();
  };
  activity->callbacks->onDestroy = [](ANativeActivity* activity) {
    delete reinterpret_cast<NativeActivity*>(activity->instance);
  };
  activity->callbacks->onSaveInstanceState = [](ANativeActivity* activity,
                                                size_t* out_size) -> void* {
    auto mapping = reinterpret_cast<NativeActivity*>(activity->instance)
                       ->OnSaveInstanceState();
    if (mapping == nullptr || mapping->GetMapping() == nullptr) {
      *out_size = 0;
      return nullptr;
    }

    // This will be `free`d by the framework.
    auto copied = malloc(mapping->GetSize());
    FML_CHECK(copied != nullptr)
        << "Allocation failure while saving instance state.";
    memcpy(copied, mapping->GetMapping(), mapping->GetSize());
    *out_size = mapping->GetSize();
    return copied;
  };
  activity->callbacks->onWindowFocusChanged = [](ANativeActivity* activity,
                                                 int has_focus) {
    reinterpret_cast<NativeActivity*>(activity->instance)
        ->OnWindowFocusChanged(has_focus);
  };
  activity->callbacks->onNativeWindowCreated = [](ANativeActivity* activity,
                                                  ANativeWindow* window) {
    reinterpret_cast<NativeActivity*>(activity->instance)
        ->OnNativeWindowCreated(window);
  };
  activity->callbacks->onNativeWindowResized = [](ANativeActivity* activity,
                                                  ANativeWindow* window) {
    reinterpret_cast<NativeActivity*>(activity->instance)
        ->OnNativeWindowResized(window);
  };
  activity->callbacks->onNativeWindowRedrawNeeded =
      [](ANativeActivity* activity, ANativeWindow* window) {
        reinterpret_cast<NativeActivity*>(activity->instance)
            ->OnNativeWindowRedrawNeeded(window);
      };
  activity->callbacks->onNativeWindowDestroyed = [](ANativeActivity* activity,
                                                    ANativeWindow* window) {
    reinterpret_cast<NativeActivity*>(activity->instance)
        ->OnNativeWindowDestroyed(window);
  };
  activity->callbacks->onInputQueueCreated = [](ANativeActivity* activity,
                                                AInputQueue* queue) {
    reinterpret_cast<NativeActivity*>(activity->instance)
        ->OnInputQueueCreated(queue);
  };
  activity->callbacks->onInputQueueDestroyed = [](ANativeActivity* activity,
                                                  AInputQueue* queue) {
    reinterpret_cast<NativeActivity*>(activity->instance)
        ->OnInputQueueDestroyed(queue);
  };
  activity->callbacks->onConfigurationChanged = [](ANativeActivity* activity) {
    reinterpret_cast<NativeActivity*>(activity->instance)
        ->OnConfigurationChanged();
  };
  activity->callbacks->onLowMemory = [](ANativeActivity* activity) {
    reinterpret_cast<NativeActivity*>(activity->instance)->OnLowMemory();
  };
}

NativeActivity::~NativeActivity() = default;

void NativeActivity::OnStart() {}

void NativeActivity::OnStop() {}

void NativeActivity::OnPause() {}

void NativeActivity::OnResume() {}

std::shared_ptr<fml::Mapping> NativeActivity::OnSaveInstanceState() {
  return nullptr;
}

void NativeActivity::OnWindowFocusChanged(bool has_focus) {}

void NativeActivity::OnNativeWindowCreated(ANativeWindow* window) {}

void NativeActivity::OnNativeWindowResized(ANativeWindow* window) {}

void NativeActivity::OnNativeWindowRedrawNeeded(ANativeWindow* window) {}

void NativeActivity::OnNativeWindowDestroyed(ANativeWindow* window) {}

void NativeActivity::OnInputQueueCreated(AInputQueue* queue) {}

void NativeActivity::OnInputQueueDestroyed(AInputQueue* queue) {}

void NativeActivity::OnConfigurationChanged() {}

void NativeActivity::OnLowMemory() {}

void NativeActivity::Terminate() {
  ANativeActivity_finish(activity_);
}

}  // namespace flutter

extern "C" __attribute__((visibility("default"))) void ANativeActivity_onCreate(
    ANativeActivity* activity,
    void* saved_state,
    size_t saved_state_size) {
  std::unique_ptr<fml::Mapping> saved_state_mapping;
  if (saved_state_size > 0u) {
    saved_state_mapping = std::make_unique<fml::MallocMapping>(
        fml::MallocMapping::Copy(saved_state, saved_state_size));
  }
  flutter::NativeActivityMain(activity, std::move(saved_state_mapping))
      .release();  // Will be freed when the frame calls the onDestroy. See the
                   // delete in that callback.
}
