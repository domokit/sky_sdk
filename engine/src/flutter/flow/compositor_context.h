// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_COMPOSITOR_CONTEXT_H_
#define FLUTTER_FLOW_COMPOSITOR_CONTEXT_H_

#include <memory>
#include <string>

#include "flutter/common/graphics/texture.h"
#include "flutter/common/macros.h"
#include "flutter/flow/diff_context.h"
#include "flutter/flow/embedded_views.h"
#include "flutter/flow/raster_cache.h"
#include "flutter/flow/stopwatch.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/raster_thread_merger.h"
#include "third_party/skia/include/core/SkCanvas.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"

namespace flutter {

class LayerTree;

// The result status of CompositorContext::ScopedFrame::Raster.
enum class RasterStatus {
  // Frame has been successfully rasterized.
  kSuccess,
  // Frame has been submited, but must be submitted again. This is only used
  // on Android when switching the background surface to FlutterImageView.
  //
  // On Android, the first frame doesn't make the image available
  // to the ImageReader right away. The second frame does.
  //
  // TODO(egarciad): https://github.com/flutter/flutter/issues/65652
  kResubmit,
  // Frame has be dropped and a new frame with the same layer tree must be
  // attempted.
  //
  // This is currently used to wait for the thread merger to merge
  // the raster and platform threads.
  //
  // Since the thread merger may be disabled, the system will proceed
  // with separate threads for rasterization and platform tasks,
  // potentially leading to different performance characteristics.
  kSkipAndRetry,
};

class FrameDamage {
 public:
  // Sets previous layer tree for calculating frame damage. If not set, entire
  // frame will be repainted.
  void SetPreviousLayerTree(const LayerTree* prev_layer_tree) {
    prev_layer_tree_ = prev_layer_tree;
  }

  // Adds additional damage (accumulated for double / triple buffering).
  // This is area that will be repainted alongside any changed part.
  void AddAdditionalDamage(const DlIRect& damage) {
    additional_damage_ = additional_damage_.Union(damage);
  }

  // Specifies clip rect alignment.
  void SetClipAlignment(int horizontal, int vertical) {
    horizontal_clip_alignment_ = horizontal;
    vertical_clip_alignment_ = vertical;
  }

  // Calculates clip rect for current rasterization. This is diff of layer tree
  // and previous layer tree + any additional provided damage.
  // If previous layer tree is not specified, clip rect will be nullopt,
  // but the paint region of layer_tree will be calculated so that it can be
  // used for diffing of subsequent frames.
  std::optional<DlRect> ComputeClipRect(flutter::LayerTree& layer_tree,
                                        bool has_raster_cache,
                                        bool impeller_enabled);

  // See Damage::frame_damage.
  std::optional<DlIRect> GetFrameDamage() const {
    return damage_ ? std::make_optional(damage_->frame_damage) : std::nullopt;
  }

  // See Damage::buffer_damage.
  std::optional<DlIRect> GetBufferDamage() {
    return (damage_ && !ignore_damage_)
               ? std::make_optional(damage_->buffer_damage)
               : std::nullopt;
  }

  // Remove reported buffer_damage to inform clients that a partial repaint
  // should not be performed on this frame.
  // frame_damage is required to correctly track accumulated damage for
  // subsequent frames.
  void Reset() { ignore_damage_ = true; }

 private:
  DlIRect additional_damage_;
  std::optional<Damage> damage_;
  const LayerTree* prev_layer_tree_ = nullptr;
  int vertical_clip_alignment_ = 1;
  int horizontal_clip_alignment_ = 1;
  bool ignore_damage_ = false;
};

class CompositorContext {
 public:
  class ScopedFrame {
   public:
    ScopedFrame(CompositorContext& context,
                GrDirectContext* gr_context,
                DlCanvas* canvas,
                ExternalViewEmbedder* view_embedder,
                const SkMatrix& root_surface_transformation,
                bool instrumentation_enabled,
                bool surface_supports_readback,
                fml::RefPtr<fml::RasterThreadMerger> raster_thread_merger,
                impeller::AiksContext* aiks_context);

    virtual ~ScopedFrame();

    DlCanvas* canvas() { return canvas_; }

    ExternalViewEmbedder* view_embedder() { return view_embedder_; }

    CompositorContext& context() const { return context_; }

    const SkMatrix& root_surface_transformation() const {
      return root_surface_transformation_;
    }

    bool surface_supports_readback() { return surface_supports_readback_; }

    GrDirectContext* gr_context() const { return gr_context_; }

    impeller::AiksContext* aiks_context() const { return aiks_context_; }

    virtual RasterStatus Raster(LayerTree& layer_tree,
                                bool ignore_raster_cache,
                                FrameDamage* frame_damage);

   private:
    void PaintLayerTreeSkia(flutter::LayerTree& layer_tree,
                            std::optional<DlRect> clip_rect,
                            bool needs_save_layer,
                            bool ignore_raster_cache);

    void PaintLayerTreeImpeller(flutter::LayerTree& layer_tree,
                                std::optional<DlRect> clip_rect,
                                bool ignore_raster_cache);

    CompositorContext& context_;
    GrDirectContext* gr_context_;
    DlCanvas* canvas_;
    impeller::AiksContext* aiks_context_;
    ExternalViewEmbedder* view_embedder_;
    const SkMatrix root_surface_transformation_;
    const bool instrumentation_enabled_;
    const bool surface_supports_readback_;
    fml::RefPtr<fml::RasterThreadMerger> raster_thread_merger_;

    FML_DISALLOW_COPY_AND_ASSIGN(ScopedFrame);
  };

  CompositorContext();

  explicit CompositorContext(Stopwatch::RefreshRateUpdater& updater);

  virtual ~CompositorContext();

  virtual std::unique_ptr<ScopedFrame> AcquireFrame(
      GrDirectContext* gr_context,
      DlCanvas* canvas,
      ExternalViewEmbedder* view_embedder,
      const SkMatrix& root_surface_transformation,
      bool instrumentation_enabled,
      bool surface_supports_readback,
      fml::RefPtr<fml::RasterThreadMerger> raster_thread_merger,
      impeller::AiksContext* aiks_context);

  void OnGrContextCreated();

  void OnGrContextDestroyed();

#if !SLIMPELLER
  RasterCache& raster_cache() { return raster_cache_; }
#endif  //  !SLIMPELLER

  std::shared_ptr<TextureRegistry> texture_registry() {
    return texture_registry_;
  }

  const Stopwatch& raster_time() const { return raster_time_; }

  Stopwatch& ui_time() { return ui_time_; }

 private:
  NOT_SLIMPELLER(RasterCache raster_cache_);
  std::shared_ptr<TextureRegistry> texture_registry_;
  Stopwatch raster_time_;
  Stopwatch ui_time_;

  /// Only used by default constructor of `CompositorContext`.
  FixedRefreshRateUpdater fixed_refresh_rate_updater_;

  void BeginFrame(ScopedFrame& frame, bool enable_instrumentation);

  void EndFrame(ScopedFrame& frame, bool enable_instrumentation);

  /// @brief  Whether Impeller shouild attempt a partial repaint.
  ///         The Impeller backend requires an additional blit pass, which may
  ///         not be worthwhile if the damage region is large.
  static bool ShouldPerformPartialRepaint(std::optional<DlRect> damage_rect,
                                          DlISize layer_tree_size);

  FML_DISALLOW_COPY_AND_ASSIGN(CompositorContext);
};

}  // namespace flutter

#endif  // FLUTTER_FLOW_COMPOSITOR_CONTEXT_H_
