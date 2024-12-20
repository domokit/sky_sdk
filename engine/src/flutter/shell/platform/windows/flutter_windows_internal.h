// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_FLUTTER_WINDOWS_INTERNAL_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_FLUTTER_WINDOWS_INTERNAL_H_

#include "flutter/shell/platform/windows/public/flutter_windows.h"

#if defined(__cplusplus)
extern "C" {
#endif

// Declare functions that are currently in-progress and shall be exposed to the
// public facing API upon completion.

// Properties for configuring a Flutter view controller.
typedef struct {
  // The view's initial width.
  int width;

  // The view's initial height.
  int height;
} FlutterDesktopViewControllerProperties;

// Creates a view for the given engine.
//
// The |engine| will be started if it is not already running.
//
// The caller owns the returned reference, and is responsible for calling
// |FlutterDesktopViewControllerDestroy|. Returns a null pointer in the event of
// an error.
//
// Unlike |FlutterDesktopViewControllerCreate|, this does *not* take ownership
// of |engine| and |FlutterDesktopEngineDestroy| must be called to destroy
// the engine.
FLUTTER_EXPORT FlutterDesktopViewControllerRef
FlutterDesktopEngineCreateViewController(
    FlutterDesktopEngineRef engine,
    const FlutterDesktopViewControllerProperties* properties);

typedef int64_t PlatformViewId;

typedef struct {
  size_t struct_size;
  HWND parent_window;
  const char* platform_view_type;
  // user_data may hold any necessary additional information for creating a new
  // platform view. For example, an instance of FlutterWindow.
  void* user_data;
  PlatformViewId platform_view_id;
} FlutterPlatformViewCreationParameters;

typedef HWND (*FlutterPlatformViewFactory)(
    const FlutterPlatformViewCreationParameters*);

typedef struct {
  size_t struct_size;
  FlutterPlatformViewFactory factory;
  void* user_data;  // Arbitrary user data supplied to the creation struct.
} FlutterPlatformViewTypeEntry;

FLUTTER_EXPORT void FlutterDesktopEngineRegisterPlatformViewType(
    FlutterDesktopEngineRef engine,
    const char* view_type_name,
    FlutterPlatformViewTypeEntry view_type);

#if defined(__cplusplus)
}
#endif

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_FLUTTER_WINDOWS_INTERNAL_H_
