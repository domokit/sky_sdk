// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_FLUTTER_WINDOWS_VIEW_CONTROLLER_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_FLUTTER_WINDOWS_VIEW_CONTROLLER_H_

#include <memory>

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/windows/flutter_windows_engine.h"
#include "flutter/shell/platform/windows/flutter_windows_view.h"

namespace flutter {

/// Controls a view that displays Flutter content.
class FlutterWindowsViewController {
 public:
  FlutterWindowsViewController(std::unique_ptr<FlutterWindowsEngine> engine,
                               std::unique_ptr<FlutterWindowsView> view)
      : engine_(std::move(engine)), view_(std::move(view)) {}

  ~FlutterWindowsViewController();

  // Destroy this view controller and its view.
  //
  // If this view controller owns the engine, the engine is also destroyed.
  void Destroy();

  FlutterWindowsEngine* engine() { return view_->GetEngine(); }
  FlutterWindowsView* view() { return view_.get(); }

 private:
  // The engine owned by this view controller, if any.
  //
  // This is used only if the view controller was created
  // using |FlutterDesktopViewControllerCreate| as that takes
  // ownership of the engine. Destroying this view controller
  // also destroys the engine.
  //
  // View controllers created using |FlutterDesktopEngineCreateViewController|
  // do not take ownership of the engine and this will be null. Destroying
  // this view controller does not destroy the engine.
  std::unique_ptr<FlutterWindowsEngine> engine_;

  std::unique_ptr<FlutterWindowsView> view_;

  FML_DISALLOW_COPY_AND_ASSIGN(FlutterWindowsViewController);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_FLUTTER_WINDOWS_VIEW_CONTROLLER_H_
