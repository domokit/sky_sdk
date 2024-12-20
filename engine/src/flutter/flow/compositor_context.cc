// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/compositor_context.h"

#include <optional>
#include <utility>
#include "flutter/flow/layers/layer_tree.h"
#include "third_party/skia/include/core/SkCanvas.h"

namespace flutter {

std::optional<DlRect> FrameDamage::ComputeClipRect(
    flutter::LayerTree& layer_tree,
    bool has_raster_cache,
    bool impeller_enabled) {
  if (layer_tree.root_layer()) {
    PaintRegionMap empty_paint_region_map;
    DiffContext context(layer_tree.frame_size(), layer_tree.paint_region_map(),
                        prev_layer_tree_ ? prev_layer_tree_->paint_region_map()
                                         : empty_paint_region_map,
                        has_raster_cache, impeller_enabled);
    context.PushCullRect(DlRect::MakeSize(layer_tree.frame_size()));
    {
      DiffContext::AutoSubtreeRestore subtree(&context);
      const Layer* prev_root_layer = nullptr;
      if (!prev_layer_tree_ ||
          prev_layer_tree_->frame_size() != layer_tree.frame_size()) {
        // If there is no previous layer tree assume the entire frame must be
        // repainted.
        context.MarkSubtreeDirty(DlRect::MakeSize(layer_tree.frame_size()));
      } else {
        prev_root_layer = prev_layer_tree_->root_layer();
      }
      layer_tree.root_layer()->Diff(&context, prev_root_layer);
    }

    damage_ =
        context.ComputeDamage(additional_damage_, horizontal_clip_alignment_,
                              vertical_clip_alignment_);
    return DlRect::Make(damage_->buffer_damage);
  }
  return std::nullopt;
}

CompositorContext::CompositorContext()
    : texture_registry_(std::make_shared<TextureRegistry>()),
      raster_time_(fixed_refresh_rate_updater_),
      ui_time_(fixed_refresh_rate_updater_) {}

CompositorContext::CompositorContext(Stopwatch::RefreshRateUpdater& updater)
    : texture_registry_(std::make_shared<TextureRegistry>()),
      raster_time_(updater),
      ui_time_(updater) {}

CompositorContext::~CompositorContext() = default;

void CompositorContext::BeginFrame(ScopedFrame& frame,
                                   bool enable_instrumentation) {
  if (enable_instrumentation) {
    raster_time_.Start();
  }
}

void CompositorContext::EndFrame(ScopedFrame& frame,
                                 bool enable_instrumentation) {
  if (enable_instrumentation) {
    raster_time_.Stop();
  }
}

std::unique_ptr<CompositorContext::ScopedFrame> CompositorContext::AcquireFrame(
    GrDirectContext* gr_context,
    DlCanvas* canvas,
    ExternalViewEmbedder* view_embedder,
    const SkMatrix& root_surface_transformation,
    bool instrumentation_enabled,
    bool surface_supports_readback,
    fml::RefPtr<fml::RasterThreadMerger>
        raster_thread_merger,  // NOLINT(performance-unnecessary-value-param)
    impeller::AiksContext* aiks_context) {
  return std::make_unique<ScopedFrame>(
      *this, gr_context, canvas, view_embedder, root_surface_transformation,
      instrumentation_enabled, surface_supports_readback, raster_thread_merger,
      aiks_context);
}

CompositorContext::ScopedFrame::ScopedFrame(
    CompositorContext& context,
    GrDirectContext* gr_context,
    DlCanvas* canvas,
    ExternalViewEmbedder* view_embedder,
    const SkMatrix& root_surface_transformation,
    bool instrumentation_enabled,
    bool surface_supports_readback,
    fml::RefPtr<fml::RasterThreadMerger> raster_thread_merger,
    impeller::AiksContext* aiks_context)
    : context_(context),
      gr_context_(gr_context),
      canvas_(canvas),
      aiks_context_(aiks_context),
      view_embedder_(view_embedder),
      root_surface_transformation_(root_surface_transformation),
      instrumentation_enabled_(instrumentation_enabled),
      surface_supports_readback_(surface_supports_readback),
      raster_thread_merger_(std::move(raster_thread_merger)) {
  context_.BeginFrame(*this, instrumentation_enabled_);
}

CompositorContext::ScopedFrame::~ScopedFrame() {
  context_.EndFrame(*this, instrumentation_enabled_);
}

RasterStatus CompositorContext::ScopedFrame::Raster(
    flutter::LayerTree& layer_tree,
    bool ignore_raster_cache,
    FrameDamage* frame_damage) {
  TRACE_EVENT0("flutter", "CompositorContext::ScopedFrame::Raster");

  std::optional<DlRect> clip_rect;
  if (frame_damage) {
    clip_rect = frame_damage->ComputeClipRect(layer_tree, !ignore_raster_cache,
                                              !gr_context_);

    if (aiks_context_ &&
        !ShouldPerformPartialRepaint(clip_rect, layer_tree.frame_size())) {
      clip_rect = std::nullopt;
      frame_damage->Reset();
    }
  }

  bool root_needs_readback = layer_tree.Preroll(
      *this, ignore_raster_cache, clip_rect ? *clip_rect : kGiantRect);
  bool needs_save_layer = root_needs_readback && !surface_supports_readback();
  PostPrerollResult post_preroll_result = PostPrerollResult::kSuccess;
  if (view_embedder_ && raster_thread_merger_) {
    post_preroll_result =
        view_embedder_->PostPrerollAction(raster_thread_merger_);
  }

  if (post_preroll_result == PostPrerollResult::kResubmitFrame) {
    return RasterStatus::kResubmit;
  }
  if (post_preroll_result == PostPrerollResult::kSkipAndRetryFrame) {
    return RasterStatus::kSkipAndRetry;
  }

  if (aiks_context_) {
    PaintLayerTreeImpeller(layer_tree, clip_rect, ignore_raster_cache);
  } else {
    PaintLayerTreeSkia(layer_tree, clip_rect, needs_save_layer,
                       ignore_raster_cache);
  }
  return RasterStatus::kSuccess;
}

void CompositorContext::ScopedFrame::PaintLayerTreeSkia(
    flutter::LayerTree& layer_tree,
    std::optional<DlRect> clip_rect,
    bool needs_save_layer,
    bool ignore_raster_cache) {
  DlAutoCanvasRestore restore(canvas(), clip_rect.has_value());

  if (canvas()) {
    if (clip_rect) {
      canvas()->ClipRect(*clip_rect);
    }

    if (needs_save_layer) {
      TRACE_EVENT0("flutter", "Canvas::saveLayer");
      SkRect bounds = SkRect::Make(ToSkISize(layer_tree.frame_size()));
      DlPaint paint;
      paint.setBlendMode(DlBlendMode::kSrc);
      canvas()->SaveLayer(&bounds, &paint);
    }
    canvas()->Clear(DlColor::kTransparent());
  }

  // The canvas()->Restore() is taken care of by the DlAutoCanvasRestore
  layer_tree.Paint(*this, ignore_raster_cache);
}

void CompositorContext::ScopedFrame::PaintLayerTreeImpeller(
    flutter::LayerTree& layer_tree,
    std::optional<DlRect> clip_rect,
    bool ignore_raster_cache) {
  if (canvas() && clip_rect) {
    canvas()->Translate(-clip_rect->GetX(), -clip_rect->GetY());
  }

  layer_tree.Paint(*this, ignore_raster_cache);
}

/// @brief The max ratio of pixel width or height to size that is dirty which
///        results in a partial repaint.
///
///        Performing a partial repaint has a small overhead - Impeller needs to
///        allocate a fairly large resolve texture for the root pass instead of
///        using the drawable texture, and a final blit must be performed. At a
///        minimum, if the damage rect is the entire buffer, we must not perform
///        a partial repaint. Beyond that, we could only experimentally
///        determine what this value should be. From looking at the Flutter
///        Gallery, we noticed that there are occassionally small partial
///        repaints which shave off trivial numbers of pixels.
constexpr float kImpellerRepaintRatio = 0.7f;

bool CompositorContext::ShouldPerformPartialRepaint(
    std::optional<DlRect> damage_rect,
    DlISize layer_tree_size) {
  if (!damage_rect.has_value()) {
    return false;
  }
  if (damage_rect->GetWidth() >= layer_tree_size.width &&
      damage_rect->GetHeight() >= layer_tree_size.height) {
    return false;
  }
  auto rx = damage_rect->GetWidth() / layer_tree_size.width;
  auto ry = damage_rect->GetHeight() / layer_tree_size.height;
  return rx <= kImpellerRepaintRatio || ry <= kImpellerRepaintRatio;
}

void CompositorContext::OnGrContextCreated() {
  texture_registry_->OnGrContextCreated();
#if !SLIMPELLER
  raster_cache_.Clear();
#endif  //  !SLIMPELLER
}

void CompositorContext::OnGrContextDestroyed() {
  texture_registry_->OnGrContextDestroyed();
#if !SLIMPELLER
  raster_cache_.Clear();
#endif  //  !SLIMPELLER
}

}  // namespace flutter
