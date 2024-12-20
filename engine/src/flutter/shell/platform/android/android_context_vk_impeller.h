// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_CONTEXT_VK_IMPELLER_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_CONTEXT_VK_IMPELLER_H_

#include "flutter/fml/concurrent_message_loop.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/native_library.h"
#include "flutter/shell/platform/android/context/android_context.h"

namespace flutter {

class AndroidContextVKImpeller : public AndroidContext {
 public:
  explicit AndroidContextVKImpeller(
      const AndroidContext::ContextSettings& settings);

  ~AndroidContextVKImpeller();

  // |AndroidContext|
  bool IsValid() const override;

 private:
  fml::RefPtr<fml::NativeLibrary> vulkan_dylib_;
  bool is_valid_ = false;

  FML_DISALLOW_COPY_AND_ASSIGN(AndroidContextVKImpeller);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_CONTEXT_VK_IMPELLER_H_
