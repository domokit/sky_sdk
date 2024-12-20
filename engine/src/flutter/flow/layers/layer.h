// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_LAYERS_LAYER_H_
#define FLUTTER_FLOW_LAYERS_LAYER_H_

#include <algorithm>
#include <memory>
#include <unordered_set>
#include <vector>

#include "flutter/common/graphics/texture.h"
#include "flutter/common/macros.h"
#include "flutter/display_list/dl_canvas.h"
#include "flutter/flow/diff_context.h"
#include "flutter/flow/embedded_views.h"
#include "flutter/flow/layers/layer_state_stack.h"
#include "flutter/flow/raster_cache.h"
#include "flutter/flow/stopwatch.h"
#include "flutter/fml/build_config.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/trace_event.h"

class GrDirectContext;

namespace flutter {

namespace testing {
class MockLayer;
}  // namespace testing

class ContainerLayer;
class DisplayListLayer;
class PerformanceOverlayLayer;
class TextureLayer;
class RasterCacheItem;

static constexpr DlRect kGiantRect = DlRect::MakeLTRB(-1E9F, -1E9F, 1E9F, 1E9F);

// This should be an exact copy of the Clip enum in painting.dart.
enum Clip { kNone, kHardEdge, kAntiAlias, kAntiAliasWithSaveLayer };

struct PrerollContext {
  NOT_SLIMPELLER(RasterCache* raster_cache);
  GrDirectContext* gr_context;
  ExternalViewEmbedder* view_embedder;
  LayerStateStack& state_stack;
  sk_sp<SkColorSpace> dst_color_space;
  bool surface_needs_readback;

  // These allow us to paint in the end of subtree Preroll.
  const Stopwatch& raster_time;
  const Stopwatch& ui_time;
  std::shared_ptr<TextureRegistry> texture_registry;

  // These allow us to track properties like elevation, opacity, and the
  // presence of a platform view during Preroll.
  bool has_platform_view = false;
  // These allow us to track properties like elevation, opacity, and the
  // presence of a texture layer during Preroll.
  bool has_texture_layer = false;

  // The list of flags that describe which rendering state attributes
  // (such as opacity, ColorFilter, ImageFilter) a given layer can
  // render itself without requiring the parent to perform a protective
  // saveLayer with those attributes.
  // For containers, the flags will be set to the intersection (logical
  // and) of all of the state bits that all of the children can render
  // or to 0 if some of the children overlap and, as such, cannot apply
  // those attributes individually and separately.
  int renderable_state_flags = 0;

  std::vector<RasterCacheItem*>* raster_cached_entries;
};

struct PaintContext {
  // When splitting the scene into multiple canvases (e.g when embedding
  // a platform view on iOS) during the paint traversal we apply any state
  // changes which affect children (i.e. saveLayer attributes) to the
  // state_stack and any local rendering state changes for leaf layers to
  // the canvas or builder.
  // When we switch a canvas or builder (when painting a PlatformViewLayer)
  // the new canvas receives all of the stateful changes from the state_stack
  // to put it into the exact same state that the outgoing canvas had at the
  // time it was swapped out.
  // The state stack lazily applies saveLayer calls to its current canvas,
  // allowing leaf layers to report that they can handle rendering some of
  // its state attributes themselves via the |applyState| method.
  LayerStateStack& state_stack;
  DlCanvas* canvas;

  // Whether current canvas is an overlay canvas. Used to determine if the
  // raster cache is painting to a surface that will be displayed above a
  // platform view, in which case it will attempt to preserve the R-Tree.
  bool rendering_above_platform_view = false;

  GrDirectContext* gr_context;
  sk_sp<SkColorSpace> dst_color_space;
  ExternalViewEmbedder* view_embedder;
  const Stopwatch& raster_time;
  const Stopwatch& ui_time;
  std::shared_ptr<TextureRegistry> texture_registry;
  NOT_SLIMPELLER(const RasterCache* raster_cache);

  bool impeller_enabled = false;
  impeller::AiksContext* aiks_context;
};

// Represents a single composited layer. Created on the UI thread but then
// subsequently used on the Rasterizer thread.
class Layer {
 public:
  // The state attribute flags that represent which attributes a
  // layer can render if it plans to use a saveLayer call in its
  // |Paint| method.
  static constexpr int kSaveLayerRenderFlags =
      LayerStateStack::kCallerCanApplyOpacity |
      LayerStateStack::kCallerCanApplyColorFilter |
      LayerStateStack::kCallerCanApplyImageFilter;

  // The state attribute flags that represent which attributes a
  // layer can render if it will be rendering its content/children
  // from a cached representation.
  static constexpr int kRasterCacheRenderFlags =
      LayerStateStack::kCallerCanApplyOpacity;

  Layer();
  virtual ~Layer();

  void AssignOldLayer(Layer* old_layer) {
    original_layer_id_ = old_layer->original_layer_id_;
  }

  // Used to establish link between old layer and new layer that replaces it.
  // If this method returns true, it is assumed that this layer replaces the old
  // layer in tree and is able to diff with it.
  virtual bool IsReplacing(DiffContext* context, const Layer* old_layer) const {
    return original_layer_id_ == old_layer->original_layer_id_;
  }

  // Performs diff with given layer
  virtual void Diff(DiffContext* context, const Layer* old_layer) {}

  // Used when diffing retained layer; In case the layer is identical, it
  // doesn't need to be diffed, but the paint region needs to be stored in diff
  // context so that it can be used in next frame
  virtual void PreservePaintRegion(DiffContext* context) {
    // retained layer means same instance so 'this' is used to index into both
    // current and old region
    context->SetLayerPaintRegion(this, context->GetOldLayerPaintRegion(this));
  }

  virtual void Preroll(PrerollContext* context) = 0;

  // Used during Preroll by layers that employ a saveLayer to manage the
  // PrerollContext settings with values affected by the saveLayer mechanism.
  // This object must be created before calling Preroll on the children to
  // set up the state for the children and then restore the state upon
  // destruction.
  class AutoPrerollSaveLayerState {
   public:
    [[nodiscard]] static AutoPrerollSaveLayerState Create(
        PrerollContext* preroll_context,
        bool save_layer_is_active = true,
        bool layer_itself_performs_readback = false);

    ~AutoPrerollSaveLayerState();

   private:
    AutoPrerollSaveLayerState(PrerollContext* preroll_context,
                              bool save_layer_is_active,
                              bool layer_itself_performs_readback);

    PrerollContext* preroll_context_;
    bool save_layer_is_active_;
    bool layer_itself_performs_readback_;

    bool prev_surface_needs_readback_;
  };

  virtual void Paint(PaintContext& context) const = 0;

  virtual void PaintChildren(PaintContext& context) const { FML_DCHECK(false); }

  bool subtree_has_platform_view() const { return subtree_has_platform_view_; }
  void set_subtree_has_platform_view(bool value) {
    subtree_has_platform_view_ = value;
  }

  // Returns the paint bounds in the layer's local coordinate system
  // as determined during Preroll().  The bounds should include any
  // transform, clip or distortions performed by the layer itself,
  // but not any similar modifications inherited from its ancestors.
  const DlRect& paint_bounds() const { return paint_bounds_; }

  // This must be set by the time Preroll() returns otherwise the layer will
  // be assumed to have empty paint bounds (paints no content).
  // The paint bounds should be independent of the context outside of this
  // layer as the layer may be painted under different conditions than
  // the Preroll context. The most common example of this condition is
  // that we might Preroll the layer with a cull_rect established by a
  // clip layer above it but then we might be asked to paint anyway if
  // another layer above us needs to cache its children. During the
  // paint operation that arises due to the caching, the clip will
  // be the bounds of the layer needing caching, not the cull_rect
  // that we saw in the overall Preroll operation.
  void set_paint_bounds(const DlRect& paint_bounds) {
    paint_bounds_ = paint_bounds;
  }

  // Determines if the layer has any content.
  bool is_empty() const { return paint_bounds_.IsEmpty(); }

  // Determines if the Paint() method is necessary based on the properties
  // of the indicated PaintContext object.
  bool needs_painting(PaintContext& context) const {
    if (subtree_has_platform_view_) {
      // Workaround for the iOS embedder. The iOS embedder expects that
      // if we preroll it, then we will later call its Paint() method.
      // Now that we preroll all layers without any culling, we may
      // call its Preroll() without calling its Paint(). For now, we
      // will not perform paint culling on any subtree that has a
      // platform view.
      // See https://github.com/flutter/flutter/issues/81419
      return true;
    }
    return !context.state_stack.painting_is_nop() &&
           !context.state_stack.content_culled(paint_bounds_);
  }

  // Propagated unique_id of the first layer in "chain" of replacement layers
  // that can be diffed.
  uint64_t original_layer_id() const { return original_layer_id_; }

  uint64_t unique_id() const { return unique_id_; }

#if !SLIMPELLER
  virtual RasterCacheKeyID caching_key_id() const {
    return RasterCacheKeyID(unique_id_, RasterCacheKeyType::kLayer);
  }
#endif  //  !SLIMPELLER
  virtual const ContainerLayer* as_container_layer() const { return nullptr; }
  virtual const DisplayListLayer* as_display_list_layer() const {
    return nullptr;
  }
  virtual const TextureLayer* as_texture_layer() const { return nullptr; }
  virtual const PerformanceOverlayLayer* as_performance_overlay_layer() const {
    return nullptr;
  }
  virtual const testing::MockLayer* as_mock_layer() const { return nullptr; }

 private:
  DlRect paint_bounds_;
  uint64_t unique_id_;
  uint64_t original_layer_id_;
  bool subtree_has_platform_view_ = false;

  static uint64_t NextUniqueID();

  FML_DISALLOW_COPY_AND_ASSIGN(Layer);
};

}  // namespace flutter

#endif  // FLUTTER_FLOW_LAYERS_LAYER_H_
