// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_ANDROID_PROC_TABLE_H_
#define FLUTTER_IMPELLER_TOOLKIT_ANDROID_PROC_TABLE_H_

#include <EGL/egl.h>
#define EGL_EGLEXT_PROTOTYPES
#include <EGL/eglext.h>
#include <android/api-level.h>
#include <android/hardware_buffer.h>
#include <android/hardware_buffer_jni.h>
#include <android/surface_control.h>
#include <android/trace.h>

#include <functional>

#include "flutter/fml/logging.h"
#include "flutter/fml/native_library.h"

namespace impeller::android {

ASurfaceTransaction* ASurfaceTransaction_fromJava(JNIEnv* env,
                                                  jobject transaction);

//------------------------------------------------------------------------------
/// @brief      The Android procs along with the device API level on which these
///             will be available. There is no checking of the actual API level
///             however (because getting the API level is itself only possible
///             on API levels 29 and above).
///
///             Take care to explicitly check for the availability of these APIs
///             at runtime before invoking them.
///
///             Typically, you'll never have to deal with the proc. table
///             directly. Instead, rely on the handle wrappers (`Choreographer`,
///             `HardwareBuffer`, etc..).
///
#define FOR_EACH_ANDROID_PROC(INVOKE)                            \
  INVOKE(AChoreographer_getInstance, 24)                         \
  INVOKE(AChoreographer_postFrameCallback, 24)                   \
  INVOKE(AChoreographer_postFrameCallback64, 29)                 \
  INVOKE(AHardwareBuffer_acquire, 26)                            \
  INVOKE(AHardwareBuffer_allocate, 26)                           \
  INVOKE(AHardwareBuffer_describe, 26)                           \
  INVOKE(AHardwareBuffer_fromHardwareBuffer, 26)                 \
  INVOKE(AHardwareBuffer_getId, 31)                              \
  INVOKE(AHardwareBuffer_isSupported, 29)                        \
  INVOKE(AHardwareBuffer_lock, 26)                               \
  INVOKE(AHardwareBuffer_release, 26)                            \
  INVOKE(AHardwareBuffer_unlock, 26)                             \
  INVOKE(ANativeWindow_acquire, 0)                               \
  INVOKE(ANativeWindow_getHeight, 0)                             \
  INVOKE(ANativeWindow_getWidth, 0)                              \
  INVOKE(ANativeWindow_release, 0)                               \
  INVOKE(ASurfaceControl_createFromWindow, 29)                   \
  INVOKE(ASurfaceControl_release, 29)                            \
  INVOKE(ASurfaceTransaction_apply, 29)                          \
  INVOKE(ASurfaceTransaction_create, 29)                         \
  INVOKE(ASurfaceTransaction_delete, 29)                         \
  INVOKE(ASurfaceTransaction_reparent, 29)                       \
  INVOKE(ASurfaceTransaction_setBuffer, 29)                      \
  INVOKE(ASurfaceTransaction_setColor, 29)                       \
  INVOKE(ASurfaceTransaction_setOnComplete, 29)                  \
  INVOKE(ASurfaceTransactionStats_getPreviousReleaseFenceFd, 29) \
  INVOKE(ASurfaceTransaction_fromJava, 34)                       \
  INVOKE(ATrace_isEnabled, 23)                                   \
  INVOKE(eglGetNativeClientBufferANDROID, 0)

template <class T>
struct AndroidProc {
  using AndroidProcType = T;

  const char* proc_name = nullptr;

  AndroidProcType* proc = nullptr;

  constexpr bool IsAvailable() const { return proc != nullptr; }

  explicit constexpr operator bool() const { return IsAvailable(); }

  template <class... Args>
  auto operator()(Args&&... args) const {
    FML_DCHECK(IsAvailable())
        << "Android method " << proc_name
        << " is not available on this device. Missing check.";
    return proc(std::forward<Args>(args)...);
  }

  void Reset() { proc = nullptr; }
};

//------------------------------------------------------------------------------
/// @brief      The table of Android procs that are resolved dynamically.
///
struct ProcTable {
  ProcTable();

  ~ProcTable();

  ProcTable(const ProcTable&) = delete;

  ProcTable& operator=(const ProcTable&) = delete;

  //----------------------------------------------------------------------------
  /// @brief      If a valid proc table could be setup. This may fail in case of
  ///             setup on non-Android platforms.
  ///
  /// @return     `true` if valid.
  ///
  bool IsValid() const;

  //----------------------------------------------------------------------------
  /// @brief      Check if tracing in enabled in the process. This call can be
  ///             made at any API level.
  ///
  /// @return     If tracing is enabled.
  ///
  bool TraceIsEnabled() const;

#define DEFINE_PROC(name, api) \
  AndroidProc<decltype(name)> name = {.proc_name = #name};
  FOR_EACH_ANDROID_PROC(DEFINE_PROC);
#undef DEFINE_PROC

 private:
  std::vector<fml::RefPtr<fml::NativeLibrary>> libraries_;
  bool is_valid_ = false;
};

const ProcTable& GetProcTable();

#ifdef TESTING
ProcTable& GetMutableProcTable();
#endif

}  // namespace impeller::android

#endif  // FLUTTER_IMPELLER_TOOLKIT_ANDROID_PROC_TABLE_H_
