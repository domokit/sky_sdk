// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_GPU_GPU_SURFACE_SOFTWARE_DELEGATE_H_
#define FLUTTER_SHELL_GPU_GPU_SURFACE_SOFTWARE_DELEGATE_H_

#include "flutter/flow/embedded_views.h"
#include "flutter/fml/macros.h"
#include "third_party/skia/include/core/SkSurface.h"

namespace flutter {

//------------------------------------------------------------------------------
/// @brief      Interface implemented by all platform surfaces that can present
///             a software backing store to the "screen". The GPU surface
///             abstraction (which abstracts the client rendering API) uses this
///             delegation pattern to tell the platform surface (which abstracts
///             how backing stores fulfilled by the selected client rendering
///             API end up on the "screen" on a particular platform) when the
///             rasterizer needs to allocate and present the software backing
///             store.
///
/// @see        |IOSSurfaceSoftware|, |AndroidSurfaceSoftware|,
///             |EmbedderSurfaceSoftware|.
///
class GPUSurfaceSoftwareDelegate {
 public:
  ~GPUSurfaceSoftwareDelegate();

  //----------------------------------------------------------------------------
  /// @brief      Called when the GPU surface needs a new buffer to render a new
  ///             frame into.
  ///
  /// @param[in]  size  The size of the frame.
  ///
  /// @return     A raster surface returned by the platform.
  ///
  virtual sk_sp<SkSurface> AcquireBackingStore(const SkISize& size) = 0;

  //----------------------------------------------------------------------------
  /// @brief      Called by the platform when a frame has been rendered into the
  ///             backing store and the platform must display it on-screen.
  ///
  /// @param[in]  backing_store  The software backing store to present.
  ///
  /// @return     Returns if the platform could present the backing store onto
  ///             the screen.
  ///
  virtual bool PresentBackingStore(sk_sp<SkSurface> backing_store) = 0;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_GPU_GPU_SURFACE_SOFTWARE_DELEGATE_H_
