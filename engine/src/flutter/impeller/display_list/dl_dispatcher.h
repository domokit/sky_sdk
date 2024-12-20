// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_DISPLAY_LIST_DL_DISPATCHER_H_
#define FLUTTER_IMPELLER_DISPLAY_LIST_DL_DISPATCHER_H_

#include <memory>

#include "flutter/display_list/dl_op_receiver.h"
#include "flutter/display_list/geometry/dl_geometry_types.h"
#include "flutter/display_list/geometry/dl_path.h"
#include "flutter/display_list/utils/dl_receiver_utils.h"
#include "fml/logging.h"
#include "impeller/display_list/aiks_context.h"
#include "impeller/display_list/canvas.h"
#include "impeller/display_list/paint.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/geometry/rect.h"

namespace impeller {

using DlScalar = flutter::DlScalar;
using DlPoint = flutter::DlPoint;
using DlRect = flutter::DlRect;
using DlIRect = flutter::DlIRect;
using DlRoundRect = flutter::DlRoundRect;
using DlPath = flutter::DlPath;

class DlDispatcherBase : public flutter::DlOpReceiver {
 public:
  // |flutter::DlOpReceiver|
  void setAntiAlias(bool aa) override;

  // |flutter::DlOpReceiver|
  void setDrawStyle(flutter::DlDrawStyle style) override;

  // |flutter::DlOpReceiver|
  void setColor(flutter::DlColor color) override;

  // |flutter::DlOpReceiver|
  void setStrokeWidth(DlScalar width) override;

  // |flutter::DlOpReceiver|
  void setStrokeMiter(DlScalar limit) override;

  // |flutter::DlOpReceiver|
  void setStrokeCap(flutter::DlStrokeCap cap) override;

  // |flutter::DlOpReceiver|
  void setStrokeJoin(flutter::DlStrokeJoin join) override;

  // |flutter::DlOpReceiver|
  void setColorSource(const flutter::DlColorSource* source) override;

  // |flutter::DlOpReceiver|
  void setColorFilter(const flutter::DlColorFilter* filter) override;

  // |flutter::DlOpReceiver|
  void setInvertColors(bool invert) override;

  // |flutter::DlOpReceiver|
  void setBlendMode(flutter::DlBlendMode mode) override;

  // |flutter::DlOpReceiver|
  void setMaskFilter(const flutter::DlMaskFilter* filter) override;

  // |flutter::DlOpReceiver|
  void setImageFilter(const flutter::DlImageFilter* filter) override;

  // |flutter::DlOpReceiver|
  void save(uint32_t total_content_depth) override;

  // |flutter::DlOpReceiver|
  void saveLayer(const DlRect& bounds,
                 const flutter::SaveLayerOptions& options,
                 uint32_t total_content_depth,
                 flutter::DlBlendMode max_content_mode,
                 const flutter::DlImageFilter* backdrop,
                 std::optional<int64_t> backdrop_id) override;

  // |flutter::DlOpReceiver|
  void restore() override;

  // |flutter::DlOpReceiver|
  void translate(DlScalar tx, DlScalar ty) override;

  // |flutter::DlOpReceiver|
  void scale(DlScalar sx, DlScalar sy) override;

  // |flutter::DlOpReceiver|
  void rotate(DlScalar degrees) override;

  // |flutter::DlOpReceiver|
  void skew(DlScalar sx, DlScalar sy) override;

  // |flutter::DlOpReceiver|
  void transform2DAffine(DlScalar mxx,
                         DlScalar mxy,
                         DlScalar mxt,
                         DlScalar myx,
                         DlScalar myy,
                         DlScalar myt) override;

  // |flutter::DlOpReceiver|
  void transformFullPerspective(DlScalar mxx,
                                DlScalar mxy,
                                DlScalar mxz,
                                DlScalar mxt,
                                DlScalar myx,
                                DlScalar myy,
                                DlScalar myz,
                                DlScalar myt,
                                DlScalar mzx,
                                DlScalar mzy,
                                DlScalar mzz,
                                DlScalar mzt,
                                DlScalar mwx,
                                DlScalar mwy,
                                DlScalar mwz,
                                DlScalar mwt) override;

  // |flutter::DlOpReceiver|
  void transformReset() override;

  // |flutter::DlOpReceiver|
  void clipRect(const DlRect& rect, ClipOp clip_op, bool is_aa) override;

  // |flutter::DlOpReceiver|
  void clipOval(const DlRect& bounds, ClipOp clip_op, bool is_aa) override;

  // |flutter::DlOpReceiver|
  void clipRoundRect(const DlRoundRect& rrect,
                     ClipOp clip_op,
                     bool is_aa) override;

  // |flutter::DlOpReceiver|
  void clipPath(const DlPath& path, ClipOp clip_op, bool is_aa) override;

  // |flutter::DlOpReceiver|
  void drawColor(flutter::DlColor color, flutter::DlBlendMode mode) override;

  // |flutter::DlOpReceiver|
  void drawPaint() override;

  // |flutter::DlOpReceiver|
  void drawLine(const DlPoint& p0, const DlPoint& p1) override;

  // |flutter::DlOpReceiver|
  void drawDashedLine(const DlPoint& p0,
                      const DlPoint& p1,
                      DlScalar on_length,
                      DlScalar off_length) override;

  // |flutter::DlOpReceiver|
  void drawRect(const DlRect& rect) override;

  // |flutter::DlOpReceiver|
  void drawOval(const DlRect& bounds) override;

  // |flutter::DlOpReceiver|
  void drawCircle(const DlPoint& center, DlScalar radius) override;

  // |flutter::DlOpReceiver|
  void drawRoundRect(const DlRoundRect& rrect) override;

  // |flutter::DlOpReceiver|
  void drawDiffRoundRect(const DlRoundRect& outer,
                         const DlRoundRect& inner) override;

  // |flutter::DlOpReceiver|
  void drawPath(const DlPath& path) override;

  // |flutter::DlOpReceiver|
  void drawArc(const DlRect& oval_bounds,
               DlScalar start_degrees,
               DlScalar sweep_degrees,
               bool use_center) override;

  // |flutter::DlOpReceiver|
  void drawPoints(PointMode mode,
                  uint32_t count,
                  const DlPoint points[]) override;

  // |flutter::DlOpReceiver|
  void drawVertices(const std::shared_ptr<flutter::DlVertices>& vertices,
                    flutter::DlBlendMode dl_mode) override;

  // |flutter::DlOpReceiver|
  void drawImage(const sk_sp<flutter::DlImage> image,
                 const DlPoint& point,
                 flutter::DlImageSampling sampling,
                 bool render_with_attributes) override;

  // |flutter::DlOpReceiver|
  void drawImageRect(const sk_sp<flutter::DlImage> image,
                     const DlRect& src,
                     const DlRect& dst,
                     flutter::DlImageSampling sampling,
                     bool render_with_attributes,
                     SrcRectConstraint constraint) override;

  // |flutter::DlOpReceiver|
  void drawImageNine(const sk_sp<flutter::DlImage> image,
                     const DlIRect& center,
                     const DlRect& dst,
                     flutter::DlFilterMode filter,
                     bool render_with_attributes) override;

  // |flutter::DlOpReceiver|
  void drawAtlas(const sk_sp<flutter::DlImage> atlas,
                 const SkRSXform xform[],
                 const DlRect tex[],
                 const flutter::DlColor colors[],
                 int count,
                 flutter::DlBlendMode mode,
                 flutter::DlImageSampling sampling,
                 const DlRect* cull_rect,
                 bool render_with_attributes) override;

  // |flutter::DlOpReceiver|
  void drawDisplayList(const sk_sp<flutter::DisplayList> display_list,
                       DlScalar opacity) override;

  // |flutter::DlOpReceiver|
  void drawTextBlob(const sk_sp<SkTextBlob> blob,
                    DlScalar x,
                    DlScalar y) override;

  // |flutter::DlOpReceiver|
  void drawTextFrame(const std::shared_ptr<impeller::TextFrame>& text_frame,
                     DlScalar x,
                     DlScalar y) override;

  // |flutter::DlOpReceiver|
  void drawShadow(const DlPath& path,
                  const flutter::DlColor color,
                  const DlScalar elevation,
                  bool transparent_occluder,
                  DlScalar dpr) override;

  virtual Canvas& GetCanvas() = 0;

 protected:
  Paint paint_;
  Matrix initial_matrix_;

  static void SimplifyOrDrawPath(Canvas& canvas,
                                 const DlPath& cache,
                                 const Paint& paint);
};

class CanvasDlDispatcher : public DlDispatcherBase {
 public:
  CanvasDlDispatcher(ContentContext& renderer,
                     RenderTarget& render_target,
                     bool has_root_backdrop_filter,
                     flutter::DlBlendMode max_root_blend_mode,
                     IRect cull_rect);

  ~CanvasDlDispatcher() = default;

  void SetBackdropData(std::unordered_map<int64_t, BackdropData> backdrop,
                       size_t backdrop_count);

  // |flutter::DlOpReceiver|
  void save() override {
    // This dispatcher should never be used with the save() variant
    // that does not include the content_depth parameter.
    FML_UNREACHABLE();
  }
  using DlDispatcherBase::save;

  // |flutter::DlOpReceiver|
  void saveLayer(const DlRect& bounds,
                 const flutter::SaveLayerOptions options,
                 const flutter::DlImageFilter* backdrop,
                 std::optional<int64_t> backdrop_id) override {
    // This dispatcher should never be used with the saveLayer() variant
    // that does not include the content_depth parameter.
    FML_UNREACHABLE();
  }
  using DlDispatcherBase::saveLayer;

  void FinishRecording() { canvas_.EndReplay(); }

  // |flutter::DlOpReceiver|
  void drawVertices(const std::shared_ptr<flutter::DlVertices>& vertices,
                    flutter::DlBlendMode dl_mode) override;

 private:
  Canvas canvas_;
  const ContentContext& renderer_;

  Canvas& GetCanvas() override;
};

/// Performs a first pass over the display list to collect infomation.
/// Collects things like text frames and backdrop filters.
class FirstPassDispatcher : public flutter::IgnoreAttributeDispatchHelper,
                            public flutter::IgnoreClipDispatchHelper,
                            public flutter::IgnoreDrawDispatchHelper {
 public:
  FirstPassDispatcher(const ContentContext& renderer,
                      const Matrix& initial_matrix,
                      const Rect cull_rect);

  ~FirstPassDispatcher();

  void save() override;

  void saveLayer(const DlRect& bounds,
                 const flutter::SaveLayerOptions options,
                 const flutter::DlImageFilter* backdrop,
                 std::optional<int64_t> backdrop_id) override;

  void restore() override;

  void translate(DlScalar tx, DlScalar ty) override;

  void scale(DlScalar sx, DlScalar sy) override;

  void rotate(DlScalar degrees) override;

  void skew(DlScalar sx, DlScalar sy) override;

  // clang-format off
  // 2x3 2D affine subset of a 4x4 transform in row major order
  void transform2DAffine(DlScalar mxx, DlScalar mxy, DlScalar mxt,
                         DlScalar myx, DlScalar myy, DlScalar myt) override;

  // full 4x4 transform in row major order
  void transformFullPerspective(
      DlScalar mxx, DlScalar mxy, DlScalar mxz, DlScalar mxt,
      DlScalar myx, DlScalar myy, DlScalar myz, DlScalar myt,
      DlScalar mzx, DlScalar mzy, DlScalar mzz, DlScalar mzt,
      DlScalar mwx, DlScalar mwy, DlScalar mwz, DlScalar mwt) override;

  void transformReset() override;

  void drawTextFrame(const std::shared_ptr<impeller::TextFrame>& text_frame,
                     DlScalar x,
                     DlScalar y) override;

  void drawDisplayList(const sk_sp<flutter::DisplayList> display_list,
                       DlScalar opacity) override;

  // |flutter::DlOpReceiver|
  void setDrawStyle(flutter::DlDrawStyle style) override;

  // |flutter::DlOpReceiver|
  void setColor(flutter::DlColor color) override;

  // |flutter::DlOpReceiver|
  void setStrokeWidth(DlScalar width) override;

  // |flutter::DlOpReceiver|
  void setStrokeMiter(DlScalar limit) override;

  // |flutter::DlOpReceiver|
  void setStrokeCap(flutter::DlStrokeCap cap) override;

  // |flutter::DlOpReceiver|
  void setStrokeJoin(flutter::DlStrokeJoin join) override;

  // |flutter::DlOpReceiver|
  void setImageFilter(const flutter::DlImageFilter* filter) override;

  std::pair<std::unordered_map<int64_t, BackdropData>, size_t> TakeBackdropData();

 private:
  const Rect GetCurrentLocalCullingBounds() const;

  const ContentContext& renderer_;
  Matrix matrix_;
  std::vector<Matrix> stack_;
  std::unordered_map<int64_t, BackdropData> backdrop_data_;
  // note: cull rects are always in the global coordinate space.
  std::vector<Rect> cull_rect_state_;
  bool has_image_filter_ = false;
  size_t backdrop_count_ = 0;
  Paint paint_;
};

/// Render the provided display list to a texture with the given size.
std::shared_ptr<Texture> DisplayListToTexture(
    const sk_sp<flutter::DisplayList>& display_list,
    ISize size,
    AiksContext& context,
    bool reset_host_buffer = true,
    bool generate_mips = false);

/// Render the provided display list to the render target.
bool RenderToOnscreen(ContentContext& context, RenderTarget render_target,
                         const sk_sp<flutter::DisplayList>& display_list,
                         SkIRect cull_rect,
                         bool reset_host_buffer);

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_DISPLAY_LIST_DL_DISPATCHER_H_
