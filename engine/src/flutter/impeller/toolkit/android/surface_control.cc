// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/android/surface_control.h"

#include "impeller/base/validation.h"
#include "impeller/toolkit/android/surface_transaction.h"

namespace impeller::android {

SurfaceControl::SurfaceControl(ANativeWindow* window, const char* debug_name) {
  if (window == nullptr) {
    VALIDATION_LOG << "Parent window of surface was null.";
    return;
  }
  if (debug_name == nullptr) {
    debug_name = "Impeller Layer";
  }
  control_.reset(
      GetProcTable().ASurfaceControl_createFromWindow(window, debug_name));
}

SurfaceControl::~SurfaceControl() {
  if (IsValid() && !RemoveFromParent()) {
    VALIDATION_LOG << "Surface control could not be removed from its parent. "
                      "Expect a leak.";
  }
}

bool SurfaceControl::IsValid() const {
  return control_.is_valid();
}

ASurfaceControl* SurfaceControl::GetHandle() const {
  return control_.get();
}

bool SurfaceControl::RemoveFromParent() const {
  if (!IsValid()) {
    return false;
  }
  SurfaceTransaction transaction;
  if (!transaction.SetParent(*this, nullptr)) {
    return false;
  }
  return transaction.Apply();
}

bool SurfaceControl::IsAvailableOnPlatform() {
  auto api_level = android_get_device_api_level();

  // Technically SurfaceControl is supported on API 29 but I've observed
  // enough reported bugs that I'm bumping the constraint to 30. If
  // we had more time to test all of these older devices maybe we could
  // figure out what the problem is.
  // https://github.com/flutter/flutter/issues/155877
  return api_level >= 30 && GetProcTable().IsValid() &&
         GetProcTable().ASurfaceControl_createFromWindow.IsAvailable();
}

}  // namespace impeller::android
