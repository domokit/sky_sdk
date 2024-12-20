// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_DIFF_CONTEXT_H_
#define FLUTTER_FLOW_DIFF_CONTEXT_H_

#include <functional>
#include <map>
#include <optional>
#include <vector>
#include "display_list/utils/dl_matrix_clip_tracker.h"
#include "flutter/flow/paint_region.h"
#include "flutter/fml/macros.h"

namespace flutter {

class Layer;

// Represents area that needs to be updated in front buffer (frame_damage) and
// area that is going to be painted to in back buffer (buffer_damage).
struct Damage {
  // This is the damage between current and previous frame;
  // If embedder supports partial update, this is the region that needs to be
  // repainted.
  // Corresponds to "surface damage" from EGL_KHR_partial_update.
  DlIRect frame_damage;

  // Reflects actual change to target framebuffer; This is frame_damage +
  // damage previously acumulated for target framebuffer.
  // All drawing will be clipped to this region. Knowing the affected area
  // upfront may be useful for tile based GPUs.
  // Corresponds to "buffer damage" from EGL_KHR_partial_update.
  DlIRect buffer_damage;
};

// Layer Unique Id to PaintRegion
using PaintRegionMap = std::map<uint64_t, PaintRegion>;

// Tracks state during tree diffing process and computes resulting damage
class DiffContext {
 public:
  explicit DiffContext(DlISize frame_size,
                       PaintRegionMap& this_frame_paint_region_map,
                       const PaintRegionMap& last_frame_paint_region_map,
                       bool has_raster_cache,
                       bool impeller_enabled);

  // Starts a new subtree.
  void BeginSubtree();

  // Ends current subtree; All modifications to state (transform, cullrect,
  // dirty) will be restored
  void EndSubtree();

  // Creates subtree in current scope and closes it on scope exit
  class AutoSubtreeRestore {
    FML_DISALLOW_COPY_ASSIGN_AND_MOVE(AutoSubtreeRestore);

   public:
    explicit AutoSubtreeRestore(DiffContext* context) : context_(context) {
      context->BeginSubtree();
    }
    ~AutoSubtreeRestore() { context_->EndSubtree(); }

   private:
    DiffContext* context_;
  };

  // Pushes additional transform for current subtree
  void PushTransform(const DlMatrix& transform);

  // Pushes cull rect for current subtree
  bool PushCullRect(const DlRect& clip);

  // Function that adjusts layer bounds (in device coordinates) depending
  // on filter.
  using FilterBoundsAdjustment = std::function<DlRect(DlRect)>;

  // Pushes filter bounds adjustment to current subtree. Every layer in this
  // subtree will have bounds adjusted by this function.
  void PushFilterBoundsAdjustment(const FilterBoundsAdjustment& filter);

  // Instruct DiffContext that current layer will paint with integral transform.
  void WillPaintWithIntegralTransform() { state_.integral_transform = true; }

  // Returns current transform as DlMatrix.
  const DlMatrix& GetMatrix() const;

  // Return cull rect for current subtree (in local coordinates).
  DlRect GetCullRect() const;

  // Sets the dirty flag on current subtree.
  //
  // previous_paint_region, which should represent region of previous subtree
  // at this level will be added to damage area.
  //
  // Each paint region added to dirty subtree (through AddPaintRegion) is also
  // added to damage.
  void MarkSubtreeDirty(
      const PaintRegion& previous_paint_region = PaintRegion());
  void MarkSubtreeDirty(const DlRect& previous_paint_region);

  bool IsSubtreeDirty() const { return state_.dirty; }

  // Marks that current subtree contains a TextureLayer. This is needed to
  // ensure that we'll Diff the TextureLayer even if inside retained layer.
  void MarkSubtreeHasTextureLayer();

  // Add layer bounds to current paint region; rect is in "local" (layer)
  // coordinates.
  void AddLayerBounds(const DlRect& rect);

  // Add entire paint region of retained layer for current subtree. This can
  // only be used in subtrees that are not dirty, otherwise ancestor transforms
  // or clips may result in different paint region.
  void AddExistingPaintRegion(const PaintRegion& region);

  // The idea of readback region is that if any part of the readback region
  // needs to be repainted, then the whole readback region must be repainted;
  //
  // paint_rect - rectangle where the filter paints contents (in screen
  //              coordinates)
  // readback_rect - rectangle where the filter samples from (in screen
  //                 coordinates)
  void AddReadbackRegion(const DlIRect& paint_rect,
                         const DlIRect& readback_rect);

  // Returns the paint region for current subtree; Each rect in paint region is
  // in screen coordinates; Once a layer accumulates the paint regions of its
  // children, this PaintRegion value can be associated with the current layer
  // using DiffContext::SetLayerPaintRegion.
  PaintRegion CurrentSubtreeRegion() const;

  // Computes final damage
  //
  // additional_damage is the previously accumulated frame_damage for
  // current framebuffer
  //
  // clip_alignment controls the alignment of resulting frame and surface
  // damage.
  Damage ComputeDamage(const DlIRect& additional_damage,
                       int horizontal_clip_alignment = 0,
                       int vertical_clip_alignment = 0) const;

  // Adds the region to current damage. Used for removed layers, where instead
  // of diffing the layer its paint region is direcly added to damage.
  void AddDamage(const PaintRegion& damage);

  // Associates the paint region with specified layer and current layer tree.
  // The paint region can not be stored directly in layer itself, because same
  // retained layer instance can possibly paint in different locations depending
  // on ancestor layers.
  void SetLayerPaintRegion(const Layer* layer, const PaintRegion& region);

  // Retrieves the paint region associated with specified layer and previous
  // frame layer tree.
  PaintRegion GetOldLayerPaintRegion(const Layer* layer) const;

  // Whether or not a raster cache is being used. If so, we must snap
  // all transformations to physical pixels if the layer may be raster
  // cached.
  bool has_raster_cache() const { return has_raster_cache_; }

  bool impeller_enabled() const { return impeller_enabled_; }

  class Statistics {
   public:
    // Picture replaced by different picture
    void AddNewPicture() { ++new_pictures_; }

    // Picture that would require deep comparison but was considered too complex
    // to serialize and thus was treated as new picture
    void AddPictureTooComplexToCompare() { ++pictures_too_complex_to_compare_; }

    // Picture that has identical instance between frames
    void AddSameInstancePicture() { ++same_instance_pictures_; };

    // Picture that had to be serialized to compare for equality
    void AddDeepComparePicture() { ++deep_compare_pictures_; }

    // Picture that had to be serialized to compare (different instances),
    // but were equal
    void AddDifferentInstanceButEqualPicture() {
      ++different_instance_but_equal_pictures_;
    };

    // Logs the statistics to trace counter
    void LogStatistics();

   private:
    int new_pictures_ = 0;
    int pictures_too_complex_to_compare_ = 0;
    int same_instance_pictures_ = 0;
    int deep_compare_pictures_ = 0;
    int different_instance_but_equal_pictures_ = 0;
  };

  Statistics& statistics() { return statistics_; }

  DlRect MapRect(const DlRect& rect);

 private:
  struct State {
    State();

    bool dirty = false;

    size_t rect_index = 0;

    // In order to replicate paint process closely, DiffContext needs to take
    // into account that some layers are painted with transform translation
    // snapped to integral coordinates.
    //
    // It's not possible to simply snap the transform itself, because culling
    // needs to happen with original (unsnapped) transform, just like it does
    // during paint. This means the integral coordinates must be applied after
    // culling before painting the layer content (either the layer itself, or
    // when starting subtree to paint layer children).
    bool integral_transform = false;

    // Current transform and clip for the layer
    DisplayListMatrixClipState matrix_clip;

    // Whether this subtree has filter bounds adjustment function. If so,
    // it will need to be removed from stack when subtree is closed.
    bool has_filter_bounds_adjustment = false;

    // Whether there is a texture layer in this subtree.
    bool has_texture = false;
  };

  void MakeTransformIntegral(DisplayListMatrixClipState& matrix_clip);

  std::shared_ptr<std::vector<DlRect>> rects_;
  State state_;
  DlISize frame_size_;
  std::vector<State> state_stack_;
  std::vector<FilterBoundsAdjustment> filter_bounds_adjustment_stack_;

  // Applies the filter bounds adjustment stack on provided rect.
  // Rect must be in device coordinates.
  DlRect ApplyFilterBoundsAdjustment(DlRect rect) const;

  DlRect damage_;

  PaintRegionMap& this_frame_paint_region_map_;
  const PaintRegionMap& last_frame_paint_region_map_;
  bool has_raster_cache_;
  bool impeller_enabled_;

  void AddDamage(const DlRect& rect);

  void AlignRect(DlIRect& rect,
                 int horizontal_alignment,
                 int vertical_clip_alignment) const;

  struct Readback {
    // Index of rects_ entry that this readback belongs to. Used to
    // determine if subtree has any readback
    size_t position;

    // Paint region of the filter performing readback, in screen coordinates.
    DlIRect paint_rect;

    // Readback area of the filter, in screen coordinates.
    DlIRect readback_rect;
  };

  std::vector<Readback> readbacks_;
  Statistics statistics_;
};

}  // namespace flutter

#endif  // FLUTTER_FLOW_DIFF_CONTEXT_H_
