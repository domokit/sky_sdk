// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_EGL_DISPLAY_H_
#define FLUTTER_IMPELLER_TOOLKIT_EGL_DISPLAY_H_

#include <memory>

#include "impeller/toolkit/egl/config.h"
#include "impeller/toolkit/egl/egl.h"

namespace impeller {
namespace egl {

class Context;
class Surface;

//------------------------------------------------------------------------------
/// @brief      A connection to an EGL display. Only one connection per
///             application instance is sufficient.
///
///             The display connection is used to first choose a config from
///             among the available, create a context from that config, and then
///             use that context with a surface on one (and only one) thread at
///             a time.
///
class Display {
 public:
  Display();

  virtual ~Display();

  //----------------------------------------------------------------------------
  /// @return     True if the display connection is valid.
  ///
  virtual bool IsValid() const;

  //----------------------------------------------------------------------------
  /// @brief      Choose a config that most closely matches a given descriptor.
  ///             If there are no matches, this method returns `nullptr`.
  ///
  /// @param[in]  config  The configuration
  ///
  /// @return     A config that matches a descriptor if one is available.
  ///             `nullptr` otherwise.
  ///
  virtual std::unique_ptr<Config> ChooseConfig(ConfigDescriptor config) const;

  //----------------------------------------------------------------------------
  /// @brief      Create a context with a supported config. The supported config
  ///             can be obtained via a successful call to `ChooseConfig`.
  ///
  /// @param[in]  config         The configuration.
  /// @param[in]  share_context  The share context. Context within the same
  ///                            share-group use the same handle table. The
  ///                            contexts should still only be used exclusively
  ///                            on each thread however.
  ///
  /// @return     A context if one can be created. `nullptr` otherwise.
  ///
  virtual std::unique_ptr<Context> CreateContext(const Config& config,
                                                 const Context* share_context);

  //----------------------------------------------------------------------------
  /// @brief      Create a window surface. The window is an opaque pointer whose
  ///             value value is platform specific. For instance, ANativeWindow
  ///             on Android.
  ///
  /// @param[in]  config  A valid configuration. One can be obtained via
  ///                     `ChooseConfig`.
  /// @param[in]  window  An opaque pointer to a platform specific window
  ///                     handle.
  ///
  /// @return     A valid window surface if one can be created. `nullptr`
  ///             otherwise.
  ///
  virtual std::unique_ptr<Surface> CreateWindowSurface(
      const Config& config,
      EGLNativeWindowType window);

  //----------------------------------------------------------------------------
  /// @brief      Create an offscreen pixelbuffer surface. These are of limited
  ///             use except in the context where applications need to render to
  ///             a texture in an offscreen context. In such cases, a 1x1 pixel
  ///             buffer surface is created to obtain a surface that can be used
  ///             to make the context current on the background thread.
  ///
  /// @param[in]  config  The configuration
  /// @param[in]  width   The width
  /// @param[in]  height  The height
  ///
  /// @return     A valid pixel buffer surface if one can be created. `nullptr`
  ///             otherwise.
  ///
  virtual std::unique_ptr<Surface>
  CreatePixelBufferSurface(const Config& config, size_t width, size_t height);

  const EGLDisplay& GetHandle() const;

 private:
  EGLDisplay display_ = EGL_NO_DISPLAY;

  Display(const Display&) = delete;

  Display& operator=(const Display&) = delete;
};

}  // namespace egl
}  // namespace impeller

#endif  // FLUTTER_IMPELLER_TOOLKIT_EGL_DISPLAY_H_
