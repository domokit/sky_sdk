// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_SURFACE_MTL_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_SURFACE_MTL_H_

#include <QuartzCore/CAMetalLayer.h>
#include <memory>

#include "impeller/geometry/rect.h"
#include "impeller/renderer/backend/metal/swapchain_transients_mtl.h"
#include "impeller/renderer/context.h"
#include "impeller/renderer/surface.h"

namespace impeller {

class SurfaceMTL final : public Surface {
 public:
#pragma GCC diagnostic push
  // Disable the diagnostic for iOS Simulators. Metal without emulation isn't
  // available prior to iOS 13 and that's what the simulator headers say when
  // support for CAMetalLayer begins. CAMetalLayer is available on iOS 8.0 and
  // above which is well below Flutters support level.
#pragma GCC diagnostic ignored "-Wunguarded-availability-new"
  //----------------------------------------------------------------------------
  /// @brief      Wraps the current drawable of the given Metal layer to create
  ///             a surface Impeller can render to. The surface must be created
  ///             as late as possible and discarded immediately after rendering
  ///             to it.
  ///
  /// @param[in]  context  The context
  /// @param[in]    layer  The layer whose current drawable to wrap to create a
  ///                      surface.
  ///
  /// @return     A pointer to the wrapped surface or null.
  ///
  static id<CAMetalDrawable> GetMetalDrawableAndValidate(
      const std::shared_ptr<Context>& context,
      CAMetalLayer* layer);

  static std::unique_ptr<SurfaceMTL> MakeFromMetalLayerDrawable(
      const std::shared_ptr<Context>& context,
      id<CAMetalDrawable> drawable,
      const std::shared_ptr<SwapchainTransientsMTL>& transients,
      std::optional<IRect> clip_rect = std::nullopt);

  static std::unique_ptr<SurfaceMTL> MakeFromTexture(
      const std::shared_ptr<Context>& context,
      id<MTLTexture> texture,
      const std::shared_ptr<SwapchainTransientsMTL>& transients,
      std::optional<IRect> clip_rect,
      id<CAMetalDrawable> drawable = nil);
#pragma GCC diagnostic pop

  // |Surface|
  ~SurfaceMTL() override;

  id<MTLDrawable> drawable() const { return drawable_; }

  // Returns a Rect defining the area of the surface in device pixels
  IRect coverage() const;

  /// Mark this surface as presenting with a transaction.
  ///
  /// If true, [Present] will block on the scheduling of a command buffer.
  void PresentWithTransaction(bool present_with_transaction) {
    present_with_transaction_ = present_with_transaction;
  }

  /// @brief Perform the final blit and trigger end of frame workloads.
  bool PreparePresent() const;

  // |Surface|
  bool Present() const override;

  void SetFrameBoundary(bool frame_boundary) {
    frame_boundary_ = frame_boundary;
  }

 private:
  std::weak_ptr<Context> context_;
  std::shared_ptr<Texture> resolve_texture_;
  id<CAMetalDrawable> drawable_ = nil;
  std::shared_ptr<Texture> source_texture_;
  std::shared_ptr<Texture> destination_texture_;
  bool requires_blit_ = false;
  std::optional<IRect> clip_rect_;
  bool frame_boundary_ = false;
  bool present_with_transaction_ = false;
  mutable bool prepared_ = false;

  static bool ShouldPerformPartialRepaint(std::optional<IRect> damage_rect);

  SurfaceMTL(const std::weak_ptr<Context>& context,
             const RenderTarget& target,
             std::shared_ptr<Texture> resolve_texture,
             id<CAMetalDrawable> drawable,
             std::shared_ptr<Texture> source_texture,
             std::shared_ptr<Texture> destination_texture,
             bool requires_blit,
             std::optional<IRect> clip_rect);

  SurfaceMTL(const SurfaceMTL&) = delete;

  SurfaceMTL& operator=(const SurfaceMTL&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_SURFACE_MTL_H_
