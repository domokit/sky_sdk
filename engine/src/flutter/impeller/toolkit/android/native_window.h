// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_ANDROID_NATIVE_WINDOW_H_
#define FLUTTER_IMPELLER_TOOLKIT_ANDROID_NATIVE_WINDOW_H_

#include "flutter/fml/unique_object.h"
#include "impeller/geometry/size.h"
#include "impeller/toolkit/android/proc_table.h"

namespace impeller::android {

//------------------------------------------------------------------------------
/// @brief      A wrapper for ANativeWindow
///             https://developer.android.com/ndk/reference/group/a-native-window
///
///             This wrapper is only available on Android.
///
class NativeWindow {
 public:
  explicit NativeWindow(ANativeWindow* window);

  ~NativeWindow();

  NativeWindow(const NativeWindow&) = delete;

  NativeWindow& operator=(const NativeWindow&) = delete;

  bool IsValid() const;

  //----------------------------------------------------------------------------
  /// @return     The current size of the native window.
  ///
  ISize GetSize() const;

  ANativeWindow* GetHandle() const;

 private:
  struct UniqueANativeWindowTraits {
    static ANativeWindow* InvalidValue() { return nullptr; }

    static bool IsValid(ANativeWindow* value) {
      return value != InvalidValue();
    }

    static void Free(ANativeWindow* value) {
      GetProcTable().ANativeWindow_release(value);
    }
  };

  fml::UniqueObject<ANativeWindow*, UniqueANativeWindowTraits> window_;
};

}  // namespace impeller::android

#endif  // FLUTTER_IMPELLER_TOOLKIT_ANDROID_NATIVE_WINDOW_H_
