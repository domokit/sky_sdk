// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_SNAPSHOT_DELEGATE_H_
#define FLUTTER_LIB_UI_SNAPSHOT_DELEGATE_H_

#include <string>

#include "flutter/common/graphics/texture.h"
#include "flutter/display_list/display_list.h"
#include "third_party/skia/include/core/SkImage.h"
#include "third_party/skia/include/gpu/ganesh/GrBackendSurface.h"
#include "third_party/skia/include/gpu/ganesh/GrContextThreadSafeProxy.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"

namespace flutter {

class DlImage;

class SnapshotDelegate {
 public:
  //----------------------------------------------------------------------------
  /// @brief      A data structure used by the Skia implementation of deferred
  ///             GPU based images.
  struct GpuImageResult {
    explicit GpuImageResult(
#if !SLIMPELLER
        const GrBackendTexture& p_texture,
#endif  //  !SLIMPELLER
        sk_sp<GrDirectContext> p_context,
        sk_sp<SkImage> p_image = nullptr,
        const std::string& p_error = "")
        :
#if !SLIMPELLER
          texture(p_texture),
#endif  //  !SLIMPELLER
          context(std::move(p_context)),
          image(std::move(p_image)),
          error(p_error) {
    }

#if !SLIMPELLER
    const GrBackendTexture texture;
#endif  //  !SLIMPELLER
    // If texture.isValid() == true, this is a pointer to a GrDirectContext that
    // can be used to create an image from the texture.
    sk_sp<GrDirectContext> context;
    // If MakeGpuImage could not create a GPU resident image, a raster copy
    // is available in this member and texture.isValid() is false.
    sk_sp<SkImage> image;

    // A non-empty string containing an error message if neither a GPU backed
    // texture nor a raster backed image could be created.
    const std::string error;
  };

  //----------------------------------------------------------------------------
  /// @brief      Attempts to create a GrBackendTexture for the specified
  ///             DisplayList. May result in a raster bitmap if no GPU context
  ///             is available.
  virtual std::unique_ptr<GpuImageResult> MakeSkiaGpuImage(
      sk_sp<DisplayList> display_list,
      const SkImageInfo& image_info) = 0;

  //----------------------------------------------------------------------------
  /// @brief      Gets the registry of external textures currently in use by the
  ///             rasterizer. These textures may be updated at a cadence
  ///             different from that of the Flutter application. When an
  ///             external texture is referenced in the Flutter layer tree, that
  ///             texture is composited within the Flutter layer tree.
  ///
  /// @return     A pointer to the external texture registry.
  ///
  virtual std::shared_ptr<TextureRegistry> GetTextureRegistry() = 0;

  virtual GrDirectContext* GetGrContext() = 0;

  virtual void MakeRasterSnapshot(
      sk_sp<DisplayList> display_list,
      SkISize picture_size,
      std::function<void(sk_sp<DlImage>)> callback) = 0;

  virtual sk_sp<DlImage> MakeRasterSnapshotSync(sk_sp<DisplayList> display_list,
                                                SkISize picture_size) = 0;

  virtual sk_sp<SkImage> ConvertToRasterImage(sk_sp<SkImage> image) = 0;

  /// Load and compile and initial PSO for the provided [runtime_stage].
  ///
  /// Impeller only.
  virtual void CacheRuntimeStage(
      const std::shared_ptr<impeller::RuntimeStage>& runtime_stage) = 0;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_SNAPSHOT_DELEGATE_H_
