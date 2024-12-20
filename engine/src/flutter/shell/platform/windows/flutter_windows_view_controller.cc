// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/flutter_windows_view_controller.h"

namespace flutter {

FlutterWindowsViewController::~FlutterWindowsViewController() {
  Destroy();
}

void FlutterWindowsViewController::Destroy() {
  if (!view_) {
    return;
  }

  // Prevent the engine from rendering into this view.
  if (view_->GetEngine()->running()) {
    auto view_id = view_->view_id();

    view_->GetEngine()->RemoveView(view_id);
  }

  // Destroy the view, followed by the engine if it is owned by this controller.
  view_.reset();
  engine_.reset();
}

}  // namespace flutter
