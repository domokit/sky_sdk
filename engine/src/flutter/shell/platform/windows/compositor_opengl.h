// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_COMPOSITOR_OPENGL_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_COMPOSITOR_OPENGL_H_

#include <memory>

#include "flutter/fml/macros.h"
#include "flutter/impeller/renderer/backend/gles/proc_table_gles.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/windows/compositor.h"
#include "flutter/shell/platform/windows/flutter_windows_engine.h"

namespace flutter {

// Enables the Flutter engine to render content on Windows using OpenGL.
class CompositorOpenGL : public Compositor {
 public:
  CompositorOpenGL(FlutterWindowsEngine* engine,
                   impeller::ProcTableGLES::Resolver resolver,
                   bool enable_impeller);

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
  // The Flutter engine that manages the views to render.
  FlutterWindowsEngine* engine_;

 private:
  struct TextureFormat {
    // The format passed to the engine using `FlutterOpenGLFramebuffer.target`.
    uint32_t sized_format = 0;
    // The format used to create textures. Passed to `glTexImage2D`.
    uint32_t general_format = 0;
  };

  // The compositor initializes itself lazily once |CreateBackingStore| is
  // called. True if initialization completed successfully.
  bool is_initialized_ = false;

  // Function used to resolve GLES functions.
  impeller::ProcTableGLES::Resolver resolver_ = nullptr;

  // Table of resolved GLES functions. Null until the compositor is initialized.
  std::unique_ptr<impeller::ProcTableGLES> gl_ = nullptr;

  // The OpenGL texture format for backing stores. Invalid value until
  // the compositor is initialized.
  TextureFormat format_;

  // Whether the Impeller rendering backend is enabled.
  bool enable_impeller_ = false;

  // Initialize the compositor. This must run on the raster thread.
  bool Initialize();

  // Clear the view's surface and removes any previously presented layers.
  bool Clear(FlutterWindowsView* view);

  FML_DISALLOW_COPY_AND_ASSIGN(CompositorOpenGL);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_COMPOSITOR_OPENGL_H_
