// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_COMPOSITOR_SOFTWARE_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_COMPOSITOR_SOFTWARE_H_

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/windows/compositor.h"
#include "flutter/shell/platform/windows/flutter_windows_engine.h"

namespace flutter {

// Enables the Flutter engine to render content on Windows using software
// rasterization and bitmaps.
class CompositorSoftware : public Compositor {
 public:
  CompositorSoftware();

  /// |Compositor|
  bool CreateBackingStore(const FlutterBackingStoreConfig& config,
                          FlutterBackingStore* result) override;
  /// |Compositor|
  bool CollectBackingStore(const FlutterBackingStore* store) override;

  /// |Compositor|
  bool Present(FlutterWindowsView* view,
               const FlutterLayer** layers,
               size_t layers_count) override;

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(CompositorSoftware);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_COMPOSITOR_SOFTWARE_H_
