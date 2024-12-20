// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_GLFW_PUBLIC_FLUTTER_GLFW_H_
#define FLUTTER_SHELL_PLATFORM_GLFW_PUBLIC_FLUTTER_GLFW_H_

#include <stddef.h>
#include <stdint.h>

#include "flutter_export.h"
#include "flutter_messenger.h"
#include "flutter_plugin_registrar.h"

#if defined(__cplusplus)
extern "C" {
#endif

// Indicates that any value is acceptable for an otherwise required property.
extern const int32_t kFlutterDesktopDontCare;

// Opaque reference to a Flutter window controller.
typedef struct FlutterDesktopWindowControllerState*
    FlutterDesktopWindowControllerRef;

// Opaque reference to a Flutter window.
typedef struct FlutterDesktopWindow* FlutterDesktopWindowRef;

// Opaque reference to a Flutter engine instance.
typedef struct FlutterDesktopEngineState* FlutterDesktopEngineRef;

// Properties representing a generic rectangular size.
typedef struct {
  int32_t width;
  int32_t height;
} FlutterDesktopSize;

// Properties for configuring a Flutter engine instance.
typedef struct {
  // The path to the flutter_assets folder for the application to be run.
  // This can either be an absolute path, or on Windows or Linux, a path
  // relative to the directory containing the executable.
  const char* assets_path;
  // The path to the icudtl.dat file for the version of Flutter you are using.
  // This can either be an absolute path, or on Windows or Linux, a path
  // relative to the directory containing the executable.
  const char* icu_data_path;
  // The path to the libapp.so file for the application to be run.
  // This can either be an absolute path or a path relative to the directory
  // containing the executable.
  const char* aot_library_path;
  // The switches to pass to the Flutter engine.
  //
  // See: https://github.com/flutter/engine/blob/main/shell/common/switches.h
  // for details. Not all arguments will apply to desktop.
  const char** switches;
  // The number of elements in |switches|.
  size_t switches_count;
} FlutterDesktopEngineProperties;

// Properties for configuring the initial settings of a Flutter window.
typedef struct {
  // The display title.
  const char* title;
  // Width in screen coordinates.
  int32_t width;
  // Height in screen coordinates.
  int32_t height;
  // Whether or not the user is prevented from resizing the window.
  // Reversed so that the default for a cleared struct is to allow resizing.
  bool prevent_resize;
} FlutterDesktopWindowProperties;

// Sets up the library's graphic context. Must be called before any other
// methods.
//
// Note: Internally, this library uses GLFW, which does not support multiple
// copies within the same process. Internally this calls glfwInit, which will
// fail if you have called glfwInit elsewhere in the process.
FLUTTER_EXPORT bool FlutterDesktopInit();

// Tears down library state. Must be called before the process terminates.
FLUTTER_EXPORT void FlutterDesktopTerminate();

// Creates a Window running a Flutter Application.
//
// FlutterDesktopInit() must be called prior to this function.
//
// This will set up and run an associated Flutter engine using the settings in
// |engine_properties|.
//
// Returns a null pointer in the event of an error. Otherwise, the pointer is
// valid until FlutterDesktopDestroyWindow is called.
// Note that calling FlutterDesktopCreateWindow without later calling
// FlutterDesktopDestroyWindow on the returned reference is a memory leak.
FLUTTER_EXPORT FlutterDesktopWindowControllerRef FlutterDesktopCreateWindow(
    const FlutterDesktopWindowProperties& window_properties,
    const FlutterDesktopEngineProperties& engine_properties);

// Shuts down the engine instance associated with |controller|, and cleans up
// associated state.
//
// |controller| is no longer valid after this call.
FLUTTER_EXPORT void FlutterDesktopDestroyWindow(
    FlutterDesktopWindowControllerRef controller);

// Waits for and processes the next event before |timeout_milliseconds|.
//
// If |timeout_milliseconds| is zero, it will wait for the next event
// indefinitely. A non-zero timeout is needed only if processing unrelated to
// the event loop is necessary (e.g., to handle events from another source).
//
// Returns false if the window should be closed as a result of the last event
// processed.
FLUTTER_EXPORT bool FlutterDesktopRunWindowEventLoopWithTimeout(
    FlutterDesktopWindowControllerRef controller,
    uint32_t timeout_milliseconds);

// Returns the window handle for the window associated with
// FlutterDesktopWindowControllerRef.
//
// Its lifetime is the same as the |controller|'s.
FLUTTER_EXPORT FlutterDesktopWindowRef FlutterDesktopGetWindow(
    FlutterDesktopWindowControllerRef controller);

// Returns the handle for the engine running in
// FlutterDesktopWindowControllerRef.
//
// Its lifetime is the same as the |controller|'s.
FLUTTER_EXPORT FlutterDesktopEngineRef FlutterDesktopGetEngine(
    FlutterDesktopWindowControllerRef controller);

// Returns the plugin registrar handle for the plugin with the given name.
//
// The name must be unique across the application.
FLUTTER_EXPORT FlutterDesktopPluginRegistrarRef
FlutterDesktopGetPluginRegistrar(FlutterDesktopEngineRef engine,
                                 const char* plugin_name);

// Enables or disables hover tracking.
//
// If hover is enabled, mouse movement will send hover events to the Flutter
// engine, rather than only tracking the mouse while the button is pressed.
// Defaults to on.
FLUTTER_EXPORT void FlutterDesktopWindowSetHoverEnabled(
    FlutterDesktopWindowRef flutter_window,
    bool enabled);

// Sets the displayed title for |flutter_window|.
FLUTTER_EXPORT void FlutterDesktopWindowSetTitle(
    FlutterDesktopWindowRef flutter_window,
    const char* title);

// Sets the displayed icon for |flutter_window|.
//
// The pixel format is 32-bit RGBA. The provided image data only needs to be
// valid for the duration of the call to this method. Pass a nullptr to revert
// to the default icon.
FLUTTER_EXPORT void FlutterDesktopWindowSetIcon(
    FlutterDesktopWindowRef flutter_window,
    uint8_t* pixel_data,
    int width,
    int height);

// Gets the position and size of |flutter_window| in screen coordinates.
FLUTTER_EXPORT void FlutterDesktopWindowGetFrame(
    FlutterDesktopWindowRef flutter_window,
    int* x,
    int* y,
    int* width,
    int* height);

// Sets the position and size of |flutter_window| in screen coordinates.
FLUTTER_EXPORT void FlutterDesktopWindowSetFrame(
    FlutterDesktopWindowRef flutter_window,
    int x,
    int y,
    int width,
    int height);

// Returns the scale factor--the number of pixels per screen coordinate--for
// |flutter_window|.
FLUTTER_EXPORT double FlutterDesktopWindowGetScaleFactor(
    FlutterDesktopWindowRef flutter_window);

// Forces a specific pixel ratio for Flutter rendering in |flutter_window|,
// rather than one computed automatically from screen information.
//
// To clear a previously set override, pass an override value of zero.
FLUTTER_EXPORT void FlutterDesktopWindowSetPixelRatioOverride(
    FlutterDesktopWindowRef flutter_window,
    double pixel_ratio);

// Sets the min/max size of |flutter_window| in screen coordinates. Use
// kFlutterDesktopDontCare for any dimension you wish to leave unconstrained.
FLUTTER_EXPORT void FlutterDesktopWindowSetSizeLimits(
    FlutterDesktopWindowRef flutter_window,
    FlutterDesktopSize minimum_size,
    FlutterDesktopSize maximum_size);

// Runs an instance of a headless Flutter engine.
//
// Returns a null pointer in the event of an error.
FLUTTER_EXPORT FlutterDesktopEngineRef FlutterDesktopRunEngine(
    const FlutterDesktopEngineProperties& properties);

// Waits for and processes the next event before |timeout_milliseconds|.
//
// If |timeout_milliseconds| is zero, it will wait for the next event
// indefinitely. A non-zero timeout is needed only if processing unrelated to
// the event loop is necessary (e.g., to handle events from another source).
FLUTTER_EXPORT void FlutterDesktopRunEngineEventLoopWithTimeout(
    FlutterDesktopEngineRef engine,
    uint32_t timeout_milliseconds);

// Shuts down the given engine instance. Returns true if the shutdown was
// successful. |engine_ref| is no longer valid after this call.
FLUTTER_EXPORT bool FlutterDesktopShutDownEngine(
    FlutterDesktopEngineRef engine);

// Returns the window associated with this registrar's engine instance.
//
// This is a GLFW shell-specific extension to flutter_plugin_registrar.h
FLUTTER_EXPORT FlutterDesktopWindowRef FlutterDesktopPluginRegistrarGetWindow(
    FlutterDesktopPluginRegistrarRef registrar);

// Enables input blocking on the given channel.
//
// If set, then the Flutter window will disable input callbacks
// while waiting for the handler for messages on that channel to run. This is
// useful if handling the message involves showing a modal window, for instance.
//
// This must be called after FlutterDesktopSetMessageHandler, as setting a
// handler on a channel will reset the input blocking state back to the
// default of disabled.
//
// This is a GLFW shell-specific extension to flutter_plugin_registrar.h
FLUTTER_EXPORT void FlutterDesktopPluginRegistrarEnableInputBlocking(
    FlutterDesktopPluginRegistrarRef registrar,
    const char* channel);

#if defined(__cplusplus)
}  // extern "C"
#endif

#endif  // FLUTTER_SHELL_PLATFORM_GLFW_PUBLIC_FLUTTER_GLFW_H_
