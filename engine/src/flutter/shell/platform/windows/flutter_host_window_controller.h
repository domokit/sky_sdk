// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_FLUTTER_HOST_WINDOW_CONTROLLER_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_FLUTTER_HOST_WINDOW_CONTROLLER_H_

#include <map>

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/method_channel.h"
#include "flutter/shell/platform/windows/flutter_host_window.h"

namespace flutter {

class FlutterWindowsEngine;

// A controller class for managing |FlutterHostWindow| instances.
// A unique instance of this class is owned by |FlutterWindowsEngine| and used
// in |WindowingHandler| to handle methods and messages enabling multi-window
// support.
class FlutterHostWindowController {
 public:
  explicit FlutterHostWindowController(FlutterWindowsEngine* engine);
  virtual ~FlutterHostWindowController() = default;

  // Creates a |FlutterHostWindow|, i.e., a native Win32 window with a
  // |FlutterWindow| parented to it. The child |FlutterWindow| implements a
  // Flutter view that is displayed in the client area of the
  // |FlutterHostWindow|.
  //
  // Returns a |WindowMetadata| with the metadata of the window just created, or
  // std::nullopt if the window could not be created.
  virtual std::optional<WindowMetadata> CreateHostWindow(
      WindowCreationSettings const& settings);

  // Creates a |FlutterHostWindow| from an existing top-level |hwnd| and |view|.
  virtual void CreateHostWindowFromExisting(HWND hwnd,
                                            FlutterWindowsView* view);

  // Modifies the attributes of the window hosting the view with ID |view_id|
  // according to the given |settings|. A "onWindowChanged" message is sent if
  // at least one attribute is modified.
  //
  // Returns false if the controller does not have a window hosting a view with
  // ID |view_id|.
  virtual bool ModifyHostWindow(
      FlutterViewId view_id,
      WindowModificationSettings const& settings) const;

  // Destroys the window that hosts the view with ID |view_id|.
  //
  // Returns false if the controller does not have a window hosting a view with
  // ID |view_id|.
  virtual bool DestroyHostWindow(FlutterViewId view_id) const;

  // Gets the window hosting the view with ID |view_id|.
  //
  // Returns nullptr if the controller does not have a window hosting a view
  // with ID |view_id|.
  FlutterHostWindow* GetHostWindow(FlutterViewId view_id) const;

  // Message handler called by |FlutterHostWindow::WndProc| to process window
  // messages before delegating them to the host window. This allows the
  // controller to process messages that affect the state of other host windows.
  LRESULT HandleMessage(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam);

  // Sets the method channel through which the controller will send the window
  // events "onWindowCreated", "onWindowDestroyed", and "onWindowChanged".
  void SetMethodChannel(std::shared_ptr<MethodChannel<EncodableValue>> channel);

  // Gets the engine that owns this controller.
  FlutterWindowsEngine* engine() const;

 private:
  // Retrieves the size of the view with ID |view_id|, in logical coordinates.
  Size GetViewSize(FlutterViewId view_id) const;

  // Sends the "onWindowChanged" message to the Flutter engine.
  void SendOnWindowChanged(FlutterViewId view_id,
                           std::optional<Size> size,
                           std::optional<Size> relative_position) const;

  // Sends the "onWindowDestroyed" message to the Flutter engine.
  void SendOnWindowDestroyed(FlutterViewId view_id) const;

  // The Flutter engine that owns this controller.
  FlutterWindowsEngine* const engine_;

  // The windowing channel through which the controller sends messages.
  std::shared_ptr<MethodChannel<EncodableValue>> channel_;

  // The host windows managed by this controller.
  std::map<FlutterViewId, std::unique_ptr<FlutterHostWindow>> windows_;

  FML_DISALLOW_COPY_AND_ASSIGN(FlutterHostWindowController);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_FLUTTER_HOST_WINDOW_CONTROLLER_H_
