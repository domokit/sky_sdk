// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_DL_BUILDER_H_
#define FLUTTER_DISPLAY_LIST_DL_BUILDER_H_

#include "flutter/display_list/display_list.h"
#include "flutter/display_list/dl_blend_mode.h"
#include "flutter/display_list/dl_canvas.h"
#include "flutter/display_list/dl_op_flags.h"
#include "flutter/display_list/dl_op_receiver.h"
#include "flutter/display_list/dl_paint.h"
#include "flutter/display_list/dl_sampling_options.h"
#include "flutter/display_list/geometry/dl_geometry_types.h"
#include "flutter/display_list/image/dl_image.h"
#include "flutter/display_list/utils/dl_accumulation_rect.h"
#include "flutter/display_list/utils/dl_comparable.h"
#include "flutter/display_list/utils/dl_matrix_clip_tracker.h"
#include "flutter/fml/macros.h"

namespace flutter {

// The primary class used to build a display list. The list of methods
// here matches the list of methods invoked on a |DlOpReceiver| combined
// with the list of methods invoked on a |DlCanvas|.
class DisplayListBuilder final : public virtual DlCanvas,
                                 public SkRefCnt,
                                 virtual DlOpReceiver,
                                 DisplayListOpFlags {
 public:
  static constexpr DlRect kMaxCullRect =
      DlRect::MakeLTRB(-1E9F, -1E9F, 1E9F, 1E9F);

  explicit DisplayListBuilder(bool prepare_rtree)
      : DisplayListBuilder(kMaxCullRect, prepare_rtree) {}

  explicit DisplayListBuilder(const DlRect& cull_rect = kMaxCullRect,
                              bool prepare_rtree = false);

  DisplayListBuilder(DlScalar width, DlScalar height)
      : DisplayListBuilder(DlRect::MakeWH(width, height)) {}

  explicit DisplayListBuilder(const SkRect& cull_rect,
                              bool prepare_rtree = false)
      : DisplayListBuilder(ToDlRect(cull_rect), prepare_rtree) {}

  ~DisplayListBuilder();

  // |DlCanvas|
  DlISize GetBaseLayerDimensions() const override;
  // |DlCanvas|
  SkImageInfo GetImageInfo() const override;

  // |DlCanvas|
  void Save() override;

  // |DlCanvas|
  void SaveLayer(const std::optional<DlRect>& bounds,
                 const DlPaint* paint = nullptr,
                 const DlImageFilter* backdrop = nullptr,
                 std::optional<int64_t> backdrop_id = std::nullopt) override;
  // |DlCanvas|
  void Restore() override;
  // |DlCanvas|
  int GetSaveCount() const override { return save_stack_.size(); }
  // |DlCanvas|
  void RestoreToCount(int restore_count) override;

  // |DlCanvas|
  void Translate(DlScalar tx, DlScalar ty) override;
  // |DlCanvas|
  void Scale(DlScalar sx, DlScalar sy) override;
  // |DlCanvas|
  void Rotate(DlScalar degrees) override;
  // |DlCanvas|
  void Skew(DlScalar sx, DlScalar sy) override;

  // clang-format off
  // 2x3 2D affine subset of a 4x4 transform in row major order
  // |DlCanvas|
  void Transform2DAffine(DlScalar mxx, DlScalar mxy, DlScalar mxt,
                         DlScalar myx, DlScalar myy, DlScalar myt) override;
  // full 4x4 transform in row major order
  // |DlCanvas|
  void TransformFullPerspective(
      DlScalar mxx, DlScalar mxy, DlScalar mxz, DlScalar mxt,
      DlScalar myx, DlScalar myy, DlScalar myz, DlScalar myt,
      DlScalar mzx, DlScalar mzy, DlScalar mzz, DlScalar mzt,
      DlScalar mwx, DlScalar mwy, DlScalar mwz, DlScalar mwt) override;
  // clang-format on
  // |DlCanvas|
  void TransformReset() override;
  // |DlCanvas|
  void Transform(const DlMatrix& matrix) override;
  // |DlCanvas|
  void SetTransform(const DlMatrix& matrix) override {
    TransformReset();
    Transform(matrix);
  }

  /// Returns the 4x4 full perspective transform representing all transform
  /// operations executed so far in this DisplayList within the enclosing
  /// save stack.
  // |DlCanvas|
  DlMatrix GetMatrix() const override { return global_state().matrix(); }

  // |DlCanvas|
  void ClipRect(const DlRect& rect,
                ClipOp clip_op = ClipOp::kIntersect,
                bool is_aa = false) override;
  // |DlCanvas|
  void ClipOval(const DlRect& bounds,
                ClipOp clip_op = ClipOp::kIntersect,
                bool is_aa = false) override;
  // |DlCanvas|
  void ClipRoundRect(const DlRoundRect& rrect,
                     ClipOp clip_op = ClipOp::kIntersect,
                     bool is_aa = false) override;
  // |DlCanvas|
  void ClipPath(const DlPath& path,
                ClipOp clip_op = ClipOp::kIntersect,
                bool is_aa = false) override;

  /// Conservative estimate of the bounds of all outstanding clip operations
  /// measured in the coordinate space within which this DisplayList will
  /// be rendered.
  // |DlCanvas|
  DlRect GetDestinationClipCoverage() const override {
    return global_state().GetDeviceCullCoverage();
  }
  /// Conservative estimate of the bounds of all outstanding clip operations
  /// transformed into the local coordinate space in which currently
  /// recorded rendering operations are interpreted.
  // |DlCanvas|
  DlRect GetLocalClipCoverage() const override {
    return global_state().GetLocalCullCoverage();
  }

  /// Return true iff the supplied bounds are easily shown to be outside
  /// of the current clip bounds. This method may conservatively return
  /// false if it cannot make the determination.
  // |DlCanvas|
  bool QuickReject(const DlRect& bounds) const override;

  // |DlCanvas|
  void DrawPaint(const DlPaint& paint) override;
  // |DlCanvas|
  void DrawColor(DlColor color, DlBlendMode mode) override;
  // |DlCanvas|
  void DrawLine(const DlPoint& p0,
                const DlPoint& p1,
                const DlPaint& paint) override;
  // |DlCanvas|
  void DrawDashedLine(const DlPoint& p0,
                      const DlPoint& p1,
                      DlScalar on_length,
                      DlScalar off_length,
                      const DlPaint& paint) override;
  // |DlCanvas|
  void DrawRect(const DlRect& rect, const DlPaint& paint) override;
  // |DlCanvas|
  void DrawOval(const DlRect& bounds, const DlPaint& paint) override;
  // |DlCanvas|
  void DrawCircle(const DlPoint& center,
                  DlScalar radius,
                  const DlPaint& paint) override;
  // |DlCanvas|
  void DrawRoundRect(const DlRoundRect& rrect, const DlPaint& paint) override;
  // |DlCanvas|
  void DrawDiffRoundRect(const DlRoundRect& outer,
                         const DlRoundRect& inner,
                         const DlPaint& paint) override;
  // |DlCanvas|
  void DrawPath(const DlPath& path, const DlPaint& paint) override;
  // |DlCanvas|
  void DrawArc(const DlRect& bounds,
               DlScalar start,
               DlScalar sweep,
               bool useCenter,
               const DlPaint& paint) override;
  // |DlCanvas|
  void DrawPoints(PointMode mode,
                  uint32_t count,
                  const DlPoint pts[],
                  const DlPaint& paint) override;
  // |DlCanvas|
  void DrawVertices(const std::shared_ptr<DlVertices>& vertices,
                    DlBlendMode mode,
                    const DlPaint& paint) override;
  // |DlCanvas|
  void DrawImage(const sk_sp<DlImage>& image,
                 const DlPoint& point,
                 DlImageSampling sampling,
                 const DlPaint* paint = nullptr) override;
  // |DlCanvas|
  void DrawImageRect(
      const sk_sp<DlImage>& image,
      const DlRect& src,
      const DlRect& dst,
      DlImageSampling sampling,
      const DlPaint* paint = nullptr,
      SrcRectConstraint constraint = SrcRectConstraint::kFast) override;
  // |DlCanvas|
  void DrawImageNine(const sk_sp<DlImage>& image,
                     const DlIRect& center,
                     const DlRect& dst,
                     DlFilterMode filter,
                     const DlPaint* paint = nullptr) override;
  // |DlCanvas|
  void DrawAtlas(const sk_sp<DlImage>& atlas,
                 const SkRSXform xform[],
                 const DlRect tex[],
                 const DlColor colors[],
                 int count,
                 DlBlendMode mode,
                 DlImageSampling sampling,
                 const DlRect* cullRect,
                 const DlPaint* paint = nullptr) override;
  // |DlCanvas|
  void DrawDisplayList(const sk_sp<DisplayList> display_list,
                       DlScalar opacity = SK_Scalar1) override;
  // |DlCanvas|
  void DrawTextBlob(const sk_sp<SkTextBlob>& blob,
                    DlScalar x,
                    DlScalar y,
                    const DlPaint& paint) override;

  void drawTextFrame(const std::shared_ptr<impeller::TextFrame>& text_frame,
                     DlScalar x,
                     DlScalar y) override;

  void DrawTextFrame(const std::shared_ptr<impeller::TextFrame>& text_frame,
                     DlScalar x,
                     DlScalar y,
                     const DlPaint& paint) override;

  // |DlCanvas|
  void DrawShadow(const DlPath& path,
                  const DlColor color,
                  const DlScalar elevation,
                  bool transparent_occluder,
                  DlScalar dpr) override;

  // |DlCanvas|
  void Flush() override {}

  sk_sp<DisplayList> Build();

  ENABLE_DL_CANVAS_BACKWARDS_COMPATIBILITY

 private:
  void Init(bool prepare_rtree);

  // This method exposes the internal stateful DlOpReceiver implementation
  // of the DisplayListBuilder, primarily for testing purposes. Its use
  // is obsolete and forbidden in every other case and is only shared to a
  // pair of "friend" accessors in the benchmark/unittest files.
  DlOpReceiver& asReceiver() { return *this; }

  friend DlOpReceiver& DisplayListBuilderBenchmarkAccessor(
      DisplayListBuilder& builder);
  friend DlOpReceiver& DisplayListBuilderTestingAccessor(
      DisplayListBuilder& builder);
  friend DlPaint DisplayListBuilderTestingAttributes(
      DisplayListBuilder& builder);
  friend int DisplayListBuilderTestingLastOpIndex(DisplayListBuilder& builder);

  void SetAttributesFromPaint(const DlPaint& paint,
                              const DisplayListAttributeFlags flags);

  // |DlOpReceiver|
  void setAntiAlias(bool aa) override {
    if (current_.isAntiAlias() != aa) {
      onSetAntiAlias(aa);
    }
  }
  // |DlOpReceiver|
  void setInvertColors(bool invert) override {
    if (current_.isInvertColors() != invert) {
      onSetInvertColors(invert);
    }
  }
  // |DlOpReceiver|
  void setStrokeCap(DlStrokeCap cap) override {
    if (current_.getStrokeCap() != cap) {
      onSetStrokeCap(cap);
    }
  }
  // |DlOpReceiver|
  void setStrokeJoin(DlStrokeJoin join) override {
    if (current_.getStrokeJoin() != join) {
      onSetStrokeJoin(join);
    }
  }
  // |DlOpReceiver|
  void setDrawStyle(DlDrawStyle style) override {
    if (current_.getDrawStyle() != style) {
      onSetDrawStyle(style);
    }
  }
  // |DlOpReceiver|
  void setStrokeWidth(float width) override {
    if (current_.getStrokeWidth() != width) {
      onSetStrokeWidth(width);
    }
  }
  // |DlOpReceiver|
  void setStrokeMiter(float limit) override {
    if (current_.getStrokeMiter() != limit) {
      onSetStrokeMiter(limit);
    }
  }
  // |DlOpReceiver|
  void setColor(DlColor color) override {
    if (current_.getColor() != color) {
      onSetColor(color);
    }
  }
  // |DlOpReceiver|
  void setBlendMode(DlBlendMode mode) override {
    if (current_.getBlendMode() != mode) {
      onSetBlendMode(mode);
    }
  }
  // |DlOpReceiver|
  void setColorSource(const DlColorSource* source) override {
    if (NotEquals(current_.getColorSource(), source)) {
      onSetColorSource(source);
    }
  }
  // |DlOpReceiver|
  void setImageFilter(const DlImageFilter* filter) override {
    if (NotEquals(current_.getImageFilter(), filter)) {
      onSetImageFilter(filter);
    }
  }
  // |DlOpReceiver|
  void setColorFilter(const DlColorFilter* filter) override {
    if (NotEquals(current_.getColorFilter(), filter)) {
      onSetColorFilter(filter);
    }
  }
  // |DlOpReceiver|
  void setMaskFilter(const DlMaskFilter* filter) override {
    if (NotEquals(current_.getMaskFilter(), filter)) {
      onSetMaskFilter(filter);
    }
  }

  DlPaint CurrentAttributes() const { return current_; }
  int LastOpIndex() const { return op_index_ - 1; }

  // |DlOpReceiver|
  void save() override { Save(); }
  // Only the |renders_with_attributes()| option will be accepted here. Any
  // other flags will be ignored and calculated anew as the DisplayList is
  // built. Alternatively, use the |saveLayer(DlRect, bool)| method.
  // |DlOpReceiver|
  void saveLayer(const DlRect& bounds,
                 const SaveLayerOptions options,
                 const DlImageFilter* backdrop,
                 std::optional<int64_t> backdrop_id) override;
  // |DlOpReceiver|
  void restore() override { Restore(); }

  // |DlOpReceiver|
  void translate(DlScalar tx, DlScalar ty) override { Translate(tx, ty); }
  // |DlOpReceiver|
  void scale(DlScalar sx, DlScalar sy) override { Scale(sx, sy); }
  // |DlOpReceiver|
  void rotate(DlScalar degrees) override { Rotate(degrees); }
  // |DlOpReceiver|
  void skew(DlScalar sx, DlScalar sy) override { Skew(sx, sy); }

  // clang-format off
  // |DlOpReceiver|
  void transform2DAffine(DlScalar mxx, DlScalar mxy, DlScalar mxt,
                         DlScalar myx, DlScalar myy, DlScalar myt) override {
    Transform2DAffine(mxx, mxy, mxt, myx, myy, myt);
  }
  // |DlOpReceiver|
  void transformFullPerspective(
      DlScalar mxx, DlScalar mxy, DlScalar mxz, DlScalar mxt,
      DlScalar myx, DlScalar myy, DlScalar myz, DlScalar myt,
      DlScalar mzx, DlScalar mzy, DlScalar mzz, DlScalar mzt,
      DlScalar mwx, DlScalar mwy, DlScalar mwz, DlScalar mwt) override {
    TransformFullPerspective(mxx, mxy, mxz, mxt,
                             myx, myy, myz, myt,
                             mzx, mzy, mzz, mzt,
                             mwx, mwy, mwz, mwt);
  }
  // clang-format off
  // |DlOpReceiver|
  void transformReset() override { TransformReset(); }

  // |DlOpReceiver|
  void clipRect(const DlRect& rect, ClipOp clip_op, bool is_aa) override {
    ClipRect(rect, clip_op, is_aa);
  }
  // |DlOpReceiver|
  void clipOval(const DlRect& bounds, ClipOp clip_op, bool is_aa) override {
    ClipOval(bounds, clip_op, is_aa);
  }
  // |DlOpReceiver|
  void clipRoundRect(const DlRoundRect& rrect,
                     ClipOp clip_op,
                     bool is_aa) override {
    ClipRoundRect(rrect, clip_op, is_aa);
  }
  // |DlOpReceiver|
  void clipPath(const DlPath& path, ClipOp clip_op, bool is_aa) override {
    ClipPath(path, clip_op, is_aa);
  }

  // |DlOpReceiver|
  void drawPaint() override;
  // |DlOpReceiver|
  void drawColor(DlColor color, DlBlendMode mode) override {
    DrawColor(color, mode);
  }
  // |DlOpReceiver|
  void drawLine(const DlPoint& p0, const DlPoint& p1) override;
  // |DlOpReceiver|
  void drawDashedLine(const DlPoint& p0,
                      const DlPoint& p1,
                      DlScalar on_length,
                      DlScalar off_length) override;
  // |DlOpReceiver|
  void drawRect(const DlRect& rect) override;
  // |DlOpReceiver|
  void drawOval(const DlRect& bounds) override;
  // |DlOpReceiver|
  void drawCircle(const DlPoint& center, DlScalar radius) override;
  // |DlOpReceiver|
  void drawRoundRect(const DlRoundRect& rrect) override;
  // |DlOpReceiver|
  void drawDiffRoundRect(const DlRoundRect& outer,
                         const DlRoundRect& inner) override;
  // |DlOpReceiver|
  void drawPath(const DlPath& path) override;
  // |DlOpReceiver|
  void drawArc(const DlRect& bounds,
               DlScalar start,
               DlScalar sweep,
               bool useCenter) override;
  // |DlOpReceiver|
  void drawPoints(PointMode mode, uint32_t count, const DlPoint pts[]) override;
  // |DlOpReceiver|
  void drawVertices(const std::shared_ptr<DlVertices>& vertices,
                    DlBlendMode mode) override;

  // |DlOpReceiver|
  void drawImage(const sk_sp<DlImage> image,
                 const DlPoint& point,
                 DlImageSampling sampling,
                 bool render_with_attributes) override;
  // |DlOpReceiver|
  void drawImageRect(
      const sk_sp<DlImage> image,
      const DlRect& src,
      const DlRect& dst,
      DlImageSampling sampling,
      bool render_with_attributes,
      SrcRectConstraint constraint = SrcRectConstraint::kFast) override;
  // |DlOpReceiver|
  void drawImageNine(const sk_sp<DlImage> image,
                     const DlIRect& center,
                     const DlRect& dst,
                     DlFilterMode filter,
                     bool render_with_attributes) override;
  // |DlOpReceiver|
  void drawAtlas(const sk_sp<DlImage> atlas,
                 const SkRSXform xform[],
                 const DlRect tex[],
                 const DlColor colors[],
                 int count,
                 DlBlendMode mode,
                 DlImageSampling sampling,
                 const DlRect* cullRect,
                 bool render_with_attributes) override;

  // |DlOpReceiver|
  void drawDisplayList(const sk_sp<DisplayList> display_list,
                       DlScalar opacity) override {
    DrawDisplayList(display_list, opacity);
  }
  // |DlOpReceiver|
  void drawTextBlob(const sk_sp<SkTextBlob> blob,
                    DlScalar x,
                    DlScalar y) override;
  // |DlOpReceiver|
  void drawShadow(const DlPath& path,
                  const DlColor color,
                  const DlScalar elevation,
                  bool transparent_occluder,
                  DlScalar dpr) override {
    DrawShadow(path, color, elevation, transparent_occluder, dpr);
  }

  void checkForDeferredSave();

  DisplayListStorage storage_;
  std::vector<size_t> offsets_;
  uint32_t render_op_count_ = 0u;
  uint32_t depth_ = 0u;
  // Most rendering ops will use 1 depth value, but some attributes may
  // require an additional depth value (due to implicit saveLayers)
  uint32_t render_op_depth_cost_ = 1u;
  DlIndex op_index_ = 0;

  // bytes and ops from |drawPicture| and |drawDisplayList|
  size_t nested_bytes_ = 0;
  uint32_t nested_op_count_ = 0;

  bool is_ui_thread_safe_ = true;

  template <typename T, typename... Args>
  void* Push(size_t extra, Args&&... args);

  struct RTreeData {
    std::vector<DlRect> rects;
    std::vector<int> indices;
  };

  struct LayerInfo {
    LayerInfo(const std::shared_ptr<DlImageFilter>& filter,
              size_t rtree_rects_start_index)
        : filter(filter),
          rtree_rects_start_index(rtree_rects_start_index) {}

    // The filter that will be applied to the contents of the saveLayer
    // when it is restored into the parent layer.
    const std::shared_ptr<DlImageFilter> filter;

    // The index of the rtree rects when the saveLayer was called, used
    // only in the case that the saveLayer has a filter so that the
    // accumulated rects can be updated in the corresponding restore call.
    const size_t rtree_rects_start_index = 0;

    // The bounds accumulator for the entire DisplayList, relative to its root
    // (not used when accumulating rects for an rtree, though)
    AccumulationRect global_space_accumulator;

    // The bounds accumulator to set/verify the bounds of the most recently
    // invoked saveLayer call, relative to the root of that saveLayer
    AccumulationRect layer_local_accumulator;

    DlBlendMode max_blend_mode = DlBlendMode::kClear;

    bool opacity_incompatible_op_detected = false;
    bool affects_transparent_layer = false;
    bool contains_backdrop_filter = false;
    bool is_unbounded = false;

    bool is_group_opacity_compatible() const {
      return !opacity_incompatible_op_detected &&
             !layer_local_accumulator.overlap_detected();
    }

    void update_blend_mode(DlBlendMode mode) {
      if (max_blend_mode < mode) {
        max_blend_mode = mode;
      }
    }
  };

  // The SaveInfo class stores internal data common to both Save and
  // SaveLayer calls
  class SaveInfo {
   public:
    // For vector reallocation calls to copy vector data
    SaveInfo(const SaveInfo& copy) = default;
    SaveInfo(SaveInfo&& copy) = default;

    // For constructor (root layer) initialization
    explicit SaveInfo(const DlRect& cull_rect)
        : is_save_layer(true),
          has_valid_clip(false),
          global_state(cull_rect),
          layer_state(cull_rect),
          layer_info(new LayerInfo(nullptr, 0u)) {}

    // For regular save calls:
    // Passing a pointer to the parent_info so as to distinguish this
    // call from the copy constructors used during vector reallocations
    explicit SaveInfo(const SaveInfo* parent_info)
        : is_save_layer(false),
          has_deferred_save_op(true),
          has_valid_clip(parent_info->has_valid_clip),
          global_state(parent_info->global_state),
          layer_state(parent_info->layer_state),
          layer_info(parent_info->layer_info) {}

    // For saveLayer calls:
    explicit SaveInfo(const SaveInfo* parent_info,
                      const std::shared_ptr<DlImageFilter>& filter,
                      int rtree_rect_index)
        : is_save_layer(true),
          has_valid_clip(false),
          global_state(parent_info->global_state),
          layer_state(kMaxCullRect),
          layer_info(new LayerInfo(filter, rtree_rect_index)) {}

    const bool is_save_layer;

    bool has_deferred_save_op = false;
    bool is_nop = false;
    bool has_valid_clip;

    // The depth when the save call is recorded, used to compute the total
    // depth of its content when the associated restore is called.
    uint32_t save_depth = 0;

    // The offset into the buffer where the associated save op is recorded
    // (which is not necessarily the same as when the Save() method is called
    // due to deferred saves)
    size_t save_offset = 0;

    // The transform and clip accumulated since the root of the DisplayList
    DisplayListMatrixClipState global_state;

    // The transform and clip accumulated since the most recent saveLayer,
    // used to compute and update its bounds when the restore is called.
    DisplayListMatrixClipState layer_state;

    std::shared_ptr<LayerInfo> layer_info;

    // Records the given bounds after transforming by the global and
    // layer matrices.
    bool AccumulateBoundsLocal(const DlRect& bounds);

    // Simply transfers the local bounds to the parent
    void TransferBoundsToParent(const SaveInfo& parent);
  };

  const DlRect original_cull_rect_;
  std::vector<SaveInfo> save_stack_;
  std::optional<RTreeData> rtree_data_;

  DlPaint current_;

  // Returns a reference to the SaveInfo structure at the top of the current
  // save_stack vector. Note that the clip and matrix state can be accessed
  // more directly through global_state() and layer_state().
  SaveInfo& current_info() { return save_stack_.back(); }
  const SaveInfo& current_info() const { return save_stack_.back(); }

  // Returns a reference to the SaveInfo structure just below the top
  // of the current save_stack state.
  SaveInfo& parent_info() { return *std::prev(save_stack_.end(), 2); }
  const SaveInfo& parent_info() const {
    return *std::prev(save_stack_.end(), 2);
  }

  // Returns a reference to the LayerInfo structure at the top of the current
  // save_stack vector. Note that the clip and matrix state can be accessed
  // more directly through global_state() and layer_state().
  LayerInfo& current_layer() { return *save_stack_.back().layer_info; }
  const LayerInfo& current_layer() const {
    return *save_stack_.back().layer_info;
  }

  // Returns a reference to the LayerInfo structure just below the top
  // of the current save_stack state.
  LayerInfo& parent_layer() {
    return *std::prev(save_stack_.end(), 2)->layer_info;
  }
  const LayerInfo& parent_layer() const {
    return *std::prev(save_stack_.end(), 2)->layer_info;
  }

  // Returns a reference to the matrix and clip state for the entire
  // DisplayList. The initial transform of this state is identity and
  // the initial cull_rect is the root original_cull_rect supplied
  // in the constructor. It is a summary of all transform and clip
  // calls that have happened since the DisplayList was created
  // (and have not yet been removed by a restore() call).
  DisplayListMatrixClipState& global_state() {
    return current_info().global_state;
  }
  const DisplayListMatrixClipState& global_state() const {
    return current_info().global_state;
  }

  // Returns a reference to the matrix and clip state relative to the
  // current layer, whether that is defined by the most recent saveLayer
  // call, or by the initial root state of the entire DisplayList for
  // calls not surrounded by a saveLayer/restore pair. It is a summary
  // of only those transform and clip calls that have happened since
  // the creation of the DisplayList or since the most recent saveLayer
  // (and have not yet been removed by a restore() call).
  DisplayListMatrixClipState& layer_local_state() {
    return current_info().layer_state;
  }
  const DisplayListMatrixClipState& layer_local_state() const {
    return current_info().layer_state;
  }

  void RestoreLayer();
  void TransferLayerBounds(const DlRect& content_bounds);
  bool AdjustRTreeRects(RTreeData& data,
                        const DlImageFilter& filter,
                        const DlMatrix& matrix,
                        const DlRect& clip,
                        size_t rect_index);

  // This flag indicates whether or not the current rendering attributes
  // are compatible with rendering ops applying an inherited opacity.
  bool current_opacity_compatibility_ = true;

  // Returns the compatibility of a given blend mode for applying an
  // inherited opacity value to modulate the visibility of the op.
  // For now we only accept SrcOver blend modes but this could be expanded
  // in the future to include other (rarely used) modes that also modulate
  // the opacity of a rendering operation at the cost of a switch statement
  // or lookup table.
  static bool IsOpacityCompatible(DlBlendMode mode) {
    return (mode == DlBlendMode::kSrcOver);
  }

  void UpdateCurrentOpacityCompatibility() {
    current_opacity_compatibility_ =             //
        current_.getColorFilter() == nullptr &&  //
        !current_.isInvertColors() &&            //
        !current_.usesRuntimeEffect() &&         //
        IsOpacityCompatible(current_.getBlendMode());
  }

  // Update the opacity compatibility flags of the current layer for an op
  // that has determined its compatibility as indicated by |compatible|.
  void UpdateLayerOpacityCompatibility(bool compatible) {
    if (!compatible) {
      current_layer().opacity_incompatible_op_detected = true;
    }
  }

  // Check for opacity compatibility for an op that may or may not use the
  // current rendering attributes as indicated by |uses_blend_attribute|.
  // If the flag is false then the rendering op will be able to substitute
  // a default Paint object with the opacity applied using the default SrcOver
  // blend mode which is always compatible with applying an inherited opacity.
  void CheckLayerOpacityCompatibility(bool uses_blend_attribute = true) {
    UpdateLayerOpacityCompatibility(!uses_blend_attribute ||
                                    current_opacity_compatibility_);
  }

  void CheckLayerOpacityHairlineCompatibility() {
    UpdateLayerOpacityCompatibility(
        current_opacity_compatibility_ &&
        (current_.getDrawStyle() == DlDrawStyle::kFill ||
         current_.getStrokeWidth() > 0));
  }

  // Check for opacity compatibility for an op that ignores the current
  // attributes and uses the indicated blend |mode| to render to the layer.
  // This is only used by |drawColor| currently.
  void CheckLayerOpacityCompatibility(DlBlendMode mode) {
    UpdateLayerOpacityCompatibility(IsOpacityCompatible(mode));
  }

  void onSetAntiAlias(bool aa);
  void onSetInvertColors(bool invert);
  void onSetStrokeCap(DlStrokeCap cap);
  void onSetStrokeJoin(DlStrokeJoin join);
  void onSetDrawStyle(DlDrawStyle style);
  void onSetStrokeWidth(DlScalar width);
  void onSetStrokeMiter(DlScalar limit);
  void onSetColor(DlColor color);
  void onSetBlendMode(DlBlendMode mode);
  void onSetColorSource(const DlColorSource* source);
  void onSetImageFilter(const DlImageFilter* filter);
  void onSetColorFilter(const DlColorFilter* filter);
  void onSetMaskFilter(const DlMaskFilter* filter);

  static DisplayListAttributeFlags FlagsForPointMode(PointMode mode);

  enum class OpResult {
    kNoEffect,
    kPreservesTransparency,
    kAffectsAll,
  };

  bool paint_nops_on_transparency();
  OpResult PaintResult(const DlPaint& paint,
                       DisplayListAttributeFlags flags = kDrawPaintFlags);

  void UpdateLayerResult(OpResult result, DlBlendMode mode) {
    switch (result) {
      case OpResult::kNoEffect:
      case OpResult::kPreservesTransparency:
        break;
      case OpResult::kAffectsAll:
        current_layer().affects_transparent_layer = true;
        break;
    }
    current_layer().update_blend_mode(mode);
  }
  void UpdateLayerResult(OpResult result, bool uses_attributes = true) {
    UpdateLayerResult(result, uses_attributes ? current_.getBlendMode()
                                              : DlBlendMode::kSrcOver);
  }

  // kAnyColor is a non-opaque and non-transparent color that will not
  // trigger any short-circuit tests about the results of a blend.
  static constexpr DlColor kAnyColor = DlColor::kMidGrey().withAlphaF(0.5f);
  static_assert(!kAnyColor.isOpaque());
  static_assert(!kAnyColor.isTransparent());
  static DlColor GetEffectiveColor(const DlPaint& paint,
                                   DisplayListAttributeFlags flags);

  // Adjusts the indicated bounds for the given flags and returns true if
  // the calculation was possible, or false if it could not be estimated.
  bool AdjustBoundsForPaint(DlRect& bounds, DisplayListAttributeFlags flags);

  // Records the fact that we encountered an op that either could not
  // estimate its bounds or that fills all of the destination space.
  bool AccumulateUnbounded(const SaveInfo& save);
  bool AccumulateUnbounded() {
    return AccumulateUnbounded(current_info());
  }

  // Records the bounds for an op after modifying them according to the
  // supplied attribute flags and transforming by the current matrix.
  bool AccumulateOpBounds(const DlRect& bounds,
                          DisplayListAttributeFlags flags) {
    DlRect safe_bounds = bounds;
    return AccumulateOpBounds(safe_bounds, flags);
  }

  // Records the bounds for an op after modifying them according to the
  // supplied attribute flags and transforming by the current matrix
  // and clipping against the current clip.
  bool AccumulateOpBounds(DlRect& bounds, DisplayListAttributeFlags flags);

  // Records the given bounds after transforming by the current matrix
  // and clipping against the current clip.
  bool AccumulateBounds(const DlRect& bounds, SaveInfo& layer, int id);
  bool AccumulateBounds(const DlRect& bounds) {
    return AccumulateBounds(bounds, current_info(), op_index_);
  }
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_DL_BUILDER_H_
