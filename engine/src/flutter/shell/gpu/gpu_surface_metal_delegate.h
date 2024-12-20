// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_GPU_GPU_SURFACE_METAL_DELEGATE_H_
#define FLUTTER_SHELL_GPU_GPU_SURFACE_METAL_DELEGATE_H_

#include <stdint.h>

#include "flutter/fml/macros.h"
#include "third_party/skia/include/core/SkSize.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/gpu/ganesh/mtl/GrMtlTypes.h"

namespace flutter {

// expected to be id<MTLDevice>
typedef void* GPUMTLDeviceHandle;

// expected to be id<MTLCommandQueues>
typedef void* GPUMTLCommandQueueHandle;

// expected to be CAMetalLayer*
typedef void* GPUCAMetalLayerHandle;

// expected to be id<MTLTexture>
typedef const void* GPUMTLTextureHandle;

typedef void (*GPUMTLDestructionCallback)(void* /* destruction_context */);

struct GPUMTLTextureInfo {
  int64_t texture_id;
  GPUMTLTextureHandle texture;
  GPUMTLDestructionCallback destruction_callback;
  void* destruction_context;
};

enum class MTLRenderTargetType { kMTLTexture, kCAMetalLayer };

//------------------------------------------------------------------------------
/// @brief      Interface implemented by all platform surfaces that can present
///             a metal backing store to the "screen". The GPU surface
///             abstraction (which abstracts the client rendering API) uses this
///             delegation pattern to tell the platform surface (which abstracts
///             how backing stores fulfilled by the selected client rendering
///             API end up on the "screen" on a particular platform) when the
///             rasterizer needs to allocate and present the software backing
///             store.
///
/// @see        |IOSurfaceMetal| and |EmbedderSurfaceMetal|.
///
class GPUSurfaceMetalDelegate {
 public:
  //------------------------------------------------------------------------------
  /// @brief Construct a new GPUSurfaceMetalDelegate object with the specified
  /// render_target type.
  ///
  /// @see |MTLRenderTargetType|
  ///
  explicit GPUSurfaceMetalDelegate(MTLRenderTargetType render_target);

  virtual ~GPUSurfaceMetalDelegate();

  //------------------------------------------------------------------------------
  /// @brief Returns the handle to the CAMetalLayer to render to. This is only
  /// called when the specified render target type is `kCAMetalLayer`.
  ///
  virtual GPUCAMetalLayerHandle GetCAMetalLayer(
      const SkISize& frame_info) const = 0;

  //------------------------------------------------------------------------------
  /// @brief Presents the drawable to the "screen". The drawable is obtained
  /// from the CAMetalLayer that given by `GetCAMetalLayer` call. This is only
  /// called when the specified render target type in `kCAMetalLayer`.
  ///
  /// @see |GPUSurfaceMetalDelegate::GetCAMetalLayer|
  ///
  virtual bool PresentDrawable(GrMTLHandle drawable) const = 0;

  virtual bool PreparePresent(GrMTLHandle drawable) const { return true; }

  //------------------------------------------------------------------------------
  /// @brief Returns the handle to the MTLTexture to render to. This is only
  /// called when the specified render target type is `kMTLTexture`.
  ///
  virtual GPUMTLTextureInfo GetMTLTexture(const SkISize& frame_info) const = 0;

  //------------------------------------------------------------------------------
  /// @brief Presents the texture with `texture_id` to the "screen".
  /// `texture_id` corresponds to a texture that has been obtained by an earlier
  /// call to `GetMTLTexture`. This is only called when the specified render
  /// target type is `kMTLTexture`.
  ///
  /// @see |GPUSurfaceMetalDelegate::GetMTLTexture|
  ///
  virtual bool PresentTexture(GPUMTLTextureInfo texture) const = 0;

  //------------------------------------------------------------------------------
  /// @brief Whether to allow drawing to the surface when the GPU is disabled
  ///
  virtual bool AllowsDrawingWhenGpuDisabled() const;

  MTLRenderTargetType GetRenderTargetType();

 private:
  const MTLRenderTargetType render_target_type_;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_GPU_GPU_SURFACE_METAL_DELEGATE_H_
